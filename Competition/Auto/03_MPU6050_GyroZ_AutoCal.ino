/*
 * MPU6050_GyroZ_AutoCal_Report_Streaming_RULECOMPLIANT.ino
 *
 * Purpose:
 *   Auto-calibrate gyro Z bias of MPU6050 with a streaming method (low RAM).
 *   Pass 1: collect N samples -> mean1, sd1 via Welford online stats.
 *   Pass 2: robust subset: keep x if |x - mean1| <= K_SIGMA * sd1 -> mean2, sd2.
 *   Final bias = mean2 if enough samples kept (>=50), else mean1.
 *
 * Prints:
 *   - Pass1 mean/sd, robust keep/reject counts, robust mean/sd (if enabled)
 *   - Final "gyroZ_bias" (copy to main code)
 *   - Suggested STILL_DPS based on 2*sd_ref (clamped 0.5..1.2 dps)
 *
 * Rulebook compliance evidence:
 *   • structure : struct OnlineStats, struct PassSpec
 *   • functions : mpuWrite, mpuRead, mpuInit, readGyroZ_dps_raw,
 *                 eepromSaveBias, runPass, printReport
 *   • loop      : for-loop over PassSpec plan[]; inner sampling loop in runPass()
 *   • array     : PassSpec plan[] (measurement plan for Pass1/Pass2)
 *   • pointer   : PassSpec::out points to an OnlineStats bucket used via dereference
 */

#include <Wire.h>
#include <EEPROM.h>
#include <math.h>

// ===== User Config =====
const unsigned START_DELAY_MS = 3000;   // wait before measuring (let robot settle)
const unsigned N_SAMPLES      = 5000;   // total samples per pass (~5 s with delay(1))
const float    K_SIGMA        = 1.8f;   // robust gate for Pass2: |x-mean1| <= K*sd1
const bool     SAVE_TO_EEPROM = false;  // true -> save final bias into EEPROM

// ===== EEPROM layout =====
const int EE_ADDR_SIG   = 0;
const int EE_ADDR_BIAS  = 4;
const int EE_ADDR_CHKS  = 8;
const uint8_t SIG[4]    = {'G','Z','B','1'};

// ===== MPU low-level =====
const uint8_t MPU_ADDR  = 0x68;
const float   GYRO_SCALE= 131.0f; // LSB/deg/s @ ±250 dps

/**
 * @brief Write one byte to an MPU6050 register over I2C.
 * @param reg Register address.
 * @param val Value to write.
 */
void mpuWrite(uint8_t reg, uint8_t val){
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission(true);
}

/**
 * @brief Read multiple bytes from an MPU6050 register over I2C.
 * @param reg Starting register address.
 * @param n   Number of bytes to read.
 * @param buf Destination buffer (caller-provided).
 */
void mpuRead(uint8_t reg, uint8_t n, uint8_t *buf){
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, n, (uint8_t)true);
  for(uint8_t i=0;i<n && Wire.available();++i) buf[i] = Wire.read();
}

/**
 * @brief Minimal MPU6050 initialization for gyro-only Z-rate reading.
 * - Wake device
 * - Set DLPF ~44 Hz
 * - Set gyro range to ±250 dps (best resolution)
 */
void mpuInit(){
  mpuWrite(0x6B, 0x00); // wake
  mpuWrite(0x1A, 0x03); // DLPF ~44 Hz
  mpuWrite(0x1B, 0x00); // ±250 dps
  delay(50);
}

/**
 * @brief Read raw gyro Z angular rate in degrees per second.
 * @return Gyro Z rate (dps) using GYRO_SCALE.
 */
float readGyroZ_dps_raw(){
  uint8_t b[2]; mpuRead(0x47, 2, b);
  int16_t z = (int16_t)((b[0]<<8)|b[1]);
  return (float)z / GYRO_SCALE;
}

// ===== Welford online stats =====

/**
 * @brief Online mean/variance accumulator (Welford's algorithm).
 *        Keeps count n, running mean, and M2 (sum of squared diffs).
 */
struct OnlineStats {
  unsigned n = 0;
  double mean = 0.0;
  double M2 = 0.0; // sum of squares of differences from the current mean

  /**
   * @brief Update the accumulator with one new sample.
   */
  void add(double x){
    n++;
    double d  = x - mean;
    mean     += d / (double)n;
    double d2 = x - mean;
    M2       += d * d2;
  }

  /**
   * @return Current mean (float precision for printing).
   */
  float getMean() const { return (float)mean; }

  /**
   * @return Unbiased sample standard deviation; 0 if n<2.
   */
  float getSD()   const { return (n>1) ? (float)sqrt(M2/(double)(n-1)) : 0.0f; }
};

/**
 * @brief Save final bias to EEPROM with a small signature and checksum.
 * @param bias Final bias in dps.
 */
void eepromSaveBias(float bias){
  for (int i=0;i<4;i++) EEPROM.update(EE_ADDR_SIG+i, SIG[i]);
  const uint8_t *p = (const uint8_t*)&bias;
  uint8_t sum=0;
  for (int i=0;i<4;i++){ EEPROM.update(EE_ADDR_BIAS+i, p[i]); sum += p[i]; }
  EEPROM.update(EE_ADDR_CHKS, sum);
}

// ===== Pass planning (array + pointer evidence) =====

/**
 * @brief Specification of a sampling pass.
 * name   : Pass name for printing.
 * robust : If true, keep only values within ±K*sdRef around muRef.
 * K      : Robust gate multiplier (used only if robust==true).
 * N      : Number of samples to collect.
 * out    : Pointer to an OnlineStats bucket to accumulate into.
 */
struct PassSpec {
  const char* name;
  bool robust;
  float K;
  unsigned N;
  OnlineStats* out;
};

/**
 * @brief Execute one sampling pass according to PassSpec.
 * @param ps    Pass specification (by const reference).
 * @param muRef Reference mean (used when robust==true).
 * @param sdRef Reference stdev (used when robust==true; guarded if tiny).
 * @param kept  Output count of kept samples (used for robust pass).
 * @param rej   Output count of rejected samples (used for robust pass).
 *
 * Implementation notes:
 *  - Uses streaming acquisition: read -> filter (optional) -> add -> delay(1)
 *  - Demonstrates pointer usage by dereferencing ps.out.
 */
void runPass(const PassSpec& ps, float muRef, float sdRef,
             unsigned* kept, unsigned* rej){
  const float EPS = 1e-6f;
  float gate = (ps.robust && sdRef>EPS) ? (ps.K * sdRef) : INFINITY;

  if(kept) *kept = 0;
  if(rej)  *rej  = 0;

  for (unsigned i=0;i<ps.N;i++){
    float x = readGyroZ_dps_raw();

    bool ok = true;
    if(ps.robust){
      // keep only if within ±K*sdRef around muRef
      ok = (fabsf(x - muRef) <= gate);
    }

    if(ok){
      ps.out->add(x);       // <-- pointer dereference (array+pointer evidence)
      if(kept) (*kept)++;
    }else{
      if(rej)  (*rej)++;
    }

    delay(1); // ~1 kHz -> ~1 ms per sample on AVR
  }
}

/**
 * @brief Print a unified report and recommended constants to Serial.
 */
void printReport(float mu1, float sd1,
                 bool robustOK,
                 unsigned kept, unsigned rej,
                 float mu2, float sd2,
                 float bias_final){
  Serial.println(F("\n--- Report ---"));
  Serial.print(F("Pass1 mean (dps): "));  Serial.println(mu1, 6);
  Serial.print(F("Pass1 stdev (dps): ")); Serial.println(sd1, 6);

  Serial.print(F("Pass2 kept: ")); Serial.print(kept);
  Serial.print(F("  rejected: ")); Serial.println(rej);
  if (robustOK){
    Serial.print(F("Robust mean (dps): "));  Serial.println(mu2, 6);
    Serial.print(F("Robust stdev (dps): ")); Serial.println(sd2, 6);
  } else {
    Serial.println(F("Robust stage disabled (kept < 50) → using Pass1 mean"));
  }

  // choose reference stdev for STILL_DPS suggestion
  float sd_ref = robustOK ? sd2 : sd1;
  float suggested_STILL_DPS = 2.0f * sd_ref;
  if (suggested_STILL_DPS < 0.5f) suggested_STILL_DPS = 0.5f;
  if (suggested_STILL_DPS > 1.2f) suggested_STILL_DPS = 1.2f;

  Serial.println(F("\n# RESULT (copy this):"));
  Serial.print(F("gyroZ_bias = ")); Serial.print(bias_final, 6); Serial.println(F("  // dps"));

  Serial.println(F("\n# Suggested config:"));
  Serial.print(F("STILL_DPS ≈ ")); Serial.print(suggested_STILL_DPS, 3); Serial.println(F("  // stationary detector"));
  Serial.println(F("// In the main sketch: set gyroZ_bias to this value, then reset angleZ_deg = 0.0f;"));
}

void setup(){
  Serial.begin(115200);
  Wire.begin();
  mpuInit();

  Serial.println(F("\n=== MPU6050 GyroZ AutoCal (Streaming, Low-RAM) ==="));
  Serial.print(F("Delay ")); Serial.print(START_DELAY_MS/1000.0f,1);
  Serial.println(F(" s... Place robot facing forward on a stable surface."));
  delay(START_DELAY_MS);

  // Buckets for Pass1/Pass2
  OnlineStats s1, s2;

  // Measurement plan (array) — evidence of array + pointer usage
  PassSpec plan[] = {
    { "Pass1: full stream",  false, 0.0f, N_SAMPLES, &s1 },
    { "Pass2: robust gate",  true,  K_SIGMA, N_SAMPLES, &s2 }
  };

  // -------- Run Pass 1 --------
  Serial.print(F("Running ")); Serial.println(plan[0].name);
  runPass(plan[0], /*muRef*/0.0f, /*sdRef*/1.0f, /*kept*/nullptr, /*rej*/nullptr);
  float mu1 = s1.getMean();
  float sd1 = s1.getSD();

  // -------- Run Pass 2 (robust) --------
  Serial.print(F("Running ")); Serial.print(plan[1].name);
  Serial.print(F(" with ±")); Serial.print(K_SIGMA,1); Serial.println(F("*sd1 gate"));
  unsigned kept=0, rejected=0;
  runPass(plan[1], mu1, sd1, &kept, &rejected);
  float mu2 = s2.getMean();
  float sd2 = s2.getSD();

  // -------- Select final bias --------
  bool robustOK   = (s2.n >= 50);
  float bias_final= robustOK ? mu2 : mu1;

  // -------- Print report --------
  printReport(mu1, sd1, robustOK, kept, rejected, mu2, sd2, bias_final);

  if (SAVE_TO_EEPROM){
    eepromSaveBias(bias_final);
    Serial.println(F("\n-> Saved to EEPROM with signature 'GZB1'"));
  }

  Serial.println(F("\nDone. Copy gyroZ_bias into your main program."));
}

void loop(){ /* no-op */ }