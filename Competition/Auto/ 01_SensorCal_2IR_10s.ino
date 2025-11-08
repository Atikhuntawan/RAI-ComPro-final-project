/*
 * 01_SensorCal_2IR_10s_namespace_RULECOMPLIANT.ino
 * - LEFT/RIGHT Sharp IR calibration for wall-follow PID
 * - 10 s prep + 10 s capture per phase (CENTER → OPEN → CLOSE)
 * - ASCII-only output for Arduino Serial Monitor
 * - Prints ready-to-paste: IR_L_CENTER, IR_R_CENTER, RATIO_OFF, RATIO_ON
 *   + Hints: ERR_DEADBAND, EMA_ALPHA
 *
 * Rulebook compliance (for code evidence):
 *   • structure : struct Stats, struct Phase
 *   • functions : analogMedian3, upd, var, sd, safeDiv, banner, countdown, acquirePhase, printStats, runCalibration
 *   • loop      : main loop over phases (for-loop) + timing loops
 *   • array     : Phase phases[] (list of calibration phases)
 *   • pointer   : Stats* inside Phase (pointers to per-phase buckets)
 */

#include <math.h>

// ---- Pins ----
#define IR_LEFT_PIN   A0
#define IR_RIGHT_PIN  A1

#ifndef LED_BUILTIN
#define LED_BUILTIN 13
#endif

namespace CAL {

  // ========================= Stats & Utilities =========================

  /**
   * @brief Robust analog read using median-of-3 to reject spikes.
   * @param pin  Analog pin to read from.
   * @return Median of three back-to-back analogRead() values.
   */
  inline int analogMedian3(uint8_t pin){
    int a=analogRead(pin), b=analogRead(pin), c=analogRead(pin);
    if(a>b){int t=a;a=b;b=t;} if(b>c){int t=b;b=c;c=t;} if(a>b){int t=a;a=b;b=t;}
    return b;
  }

  /**
   * @brief Online mean/variance accumulator (Welford's algorithm).
   * Stores count, mean, and M2 (sum of squared diffs) with min/max tracking.
   */
  struct Stats {
    unsigned long n=0;
    double mean=0.0, M2=0.0;
    int vmin=1023, vmax=0;
  };

  /**
   * @brief Update Stats with a new sample.
   * @param s Stats bucket to update (passed by reference).
   * @param x New integer sample value.
   */
  inline void upd(Stats &s, int x){
    s.n++;
    double dx = x - s.mean;
    s.mean   += dx/(double)s.n;
    s.M2     += dx*(x - s.mean);
    if(x<s.vmin) s.vmin=x;
    if(x>s.vmax) s.vmax=x;
  }

  /**
   * @brief Sample variance (unbiased) from Stats.
   * @return Variance; 0 if n<2.
   */
  inline double var(const Stats&s){ return (s.n>1)? s.M2/(double)(s.n-1) : 0.0; }

  /**
   * @brief Sample standard deviation from Stats.
   * @return Standard deviation; 0 if n<2.
   */
  inline double sd (const Stats&s){ double v=var(s); return v>0? sqrt(v):0.0; }

  /**
   * @brief Safe division with small-denominator guard.
   */
  inline double safeDiv(double a,double b){ return (b>1e-6)? a/b : 0.0; }

  // ========================= Timing & I/O Helpers =========================

  // Global timing knobs
  const unsigned PREP_MS      = 10000; // 10 s prep between phases
  const unsigned SAMPLE_DT_MS = 20;    // ~50 Hz sample interval

  /**
   * @brief Print a big ASCII banner title.
   * @param t Title text.
   */
  void banner(const char* t){
    Serial.println(); Serial.println("======================================");
    Serial.println(t);
    Serial.println("======================================");
  }

  /**
   * @brief Visible countdown with onboard LED blink.
   * @param ms_total Countdown duration in milliseconds.
   */
  void countdown(unsigned ms_total){
    unsigned s = ms_total/1000;
    for(int i=(int)s;i>=1;i--){
      Serial.print(i); Serial.println("...");
      digitalWrite(LED_BUILTIN,HIGH); delay(150);
      digitalWrite(LED_BUILTIN,LOW);  delay(850);
    }
  }

  // ========================= Phase Definition (array + pointers) =========================

  // Per-phase buckets
  Stats L_center, R_center;
  Stats L_open,   R_open;
  Stats L_close,  R_close;

  /**
   * @brief A calibration phase descriptor (demonstrates structure + pointers).
   * name      : Phase name shown in console.
   * prepHint  : Instruction shown before capture (during PREP_MS).
   * phaseHint : Instruction during capture.
   * L, R      : Pointers to Stats buckets to fill for LEFT/RIGHT sensors.
   * ms_len    : Capture length in milliseconds.
   */
  struct Phase {
    const char* name;
    const char* prepHint;
    const char* phaseHint;
    Stats* L;
    Stats* R;
    unsigned ms_len;
  };

  // 10 s capture per phase (can be tuned per phase if needed)
  const unsigned T_CENTER_MS = 10000;
  const unsigned T_OPEN_MS   = 10000;
  const unsigned T_CLOSE_MS  = 10000;

  // Phase array (demonstrates "array" requirement)
  Phase phases[] = {
    { "CENTER",
      "Prep CENTER (place robot at lane center, walls on both sides)",
      "Phase 1: CENTER (10s) - do not move",
      &L_center, &R_center, T_CENTER_MS
    },
    { "OPEN",
      "Prep OPEN (aim sensors to empty space / far from walls)",
      "Phase 2: OPEN (10s) - keep empty view",
      &L_open, &R_open, T_OPEN_MS
    },
    { "CLOSE",
      "Prep CLOSE (bring a wall/hand close ~7-10 cm, hold steady)",
      "Phase 3: CLOSE (10s) - hold distance steady",
      &L_close, &R_close, T_CLOSE_MS
    }
  };

  /**
   * @brief Acquire samples for one phase into its Stats buckets.
   * @param Ls  Stats for LEFT sensor (by reference).
   * @param Rs  Stats for RIGHT sensor (by reference).
   * @param ms_len Capture duration in milliseconds.
   */
  void acquirePhase(Stats &Ls, Stats &Rs, unsigned ms_len){
    unsigned long t0=millis();
    while(millis()-t0 < ms_len){
      int L = analogMedian3(IR_LEFT_PIN);
      int R = analogMedian3(IR_RIGHT_PIN);
      upd(Ls,L); upd(Rs,R);
      delay(SAMPLE_DT_MS);
    }
  }

  /**
   * @brief Print summary line for one Stats bucket.
   * @param name Label to print (e.g., "CENTER L:").
   * @param S    Stats to display.
   */
  void printStats(const char* name, const Stats &S){
    Serial.print(name);
    Serial.print(" n=");    Serial.print(S.n);
    Serial.print(" mean="); Serial.print(S.mean,1);
    Serial.print(" sd=");   Serial.print(sd(S),1);
    Serial.print(" min/max="); Serial.print(S.vmin); Serial.print("/"); Serial.println(S.vmax);
  }

  /**
   * @brief Run the full 3-phase calibration sequence.
   * Orchestrates PREP → CAPTURE for each Phase, then prints suggested constants.
   */
  void runCalibration(){
    banner("IR 2-Sensor Auto Calibration (ASCII)");
    Serial.println("Sequence: [Prep 10s] CENTER 10s -> [Prep 10s] OPEN 10s -> [Prep 10s] CLOSE 10s");

    // Iterate all phases (loop over array of structures using pointers)
    const int PHASE_COUNT = sizeof(phases)/sizeof(phases[0]);
    for(int i=0;i<PHASE_COUNT;i++){
      Phase &ph = phases[i];

      // Prep
      banner(ph.prepHint);
      countdown(PREP_MS);

      // Capture
      banner(ph.phaseHint);
      digitalWrite(LED_BUILTIN,HIGH);
      acquirePhase(*ph.L, *ph.R, ph.ms_len);   // <-- pointers to Stats used here
      digitalWrite(LED_BUILTIN,LOW);
    }

    // ---- Summary (means/sd/min/max) ----
    banner("Stats (mean/sd/min/max)");
    printStats("CENTER L:",L_center); printStats("CENTER R:",R_center);
    printStats("OPEN   L:",L_open);   printStats("OPEN   R:",R_open);
    printStats("CLOSE  L:",L_close);  printStats("CLOSE  R:",R_close);

    // ---- Suggestions ----
    int IR_L_CENTER_sug = (int)round(L_center.mean);
    int IR_R_CENTER_sug = (int)round(R_center.mean);

    double rL_open  = safeDiv(L_open.mean ,  L_center.mean);
    double rR_open  = safeDiv(R_open.mean ,  R_center.mean);
    double rL_close = safeDiv(L_close.mean,  L_center.mean);
    double rR_close = safeDiv(R_close.mean,  R_center.mean);
    double r_open_mean = (rL_open + rR_open)*0.5;

    // RATIO OFF/ON auto-pick with guards
    const double OFF_min=0.40, OFF_max=0.75;
    const double ON_gap_min=0.08, ON_gap_max=0.15;
    bool openLooksBad = (r_open_mean > 0.80);  // OPEN too similar to CENTER
    double RATIO_OFF_sug = 0.60, RATIO_ON_sug = 0.70;

    if(!openLooksBad){
      double sL_open = sd(L_open) /  (L_open.mean  >1e-6 ? L_open.mean  : 1);
      double sR_open = sd(R_open) /  (R_open.mean  >1e-6 ? R_open.mean  : 1);
      double s_open  = (sL_open + sR_open)*0.5;
      double margin  = 0.05 + 1.5*s_open;

      RATIO_OFF_sug = r_open_mean + margin;
      if(RATIO_OFF_sug < OFF_min) RATIO_OFF_sug = OFF_min;
      if(RATIO_OFF_sug > OFF_max) RATIO_OFF_sug = OFF_max;

      RATIO_ON_sug  = RATIO_OFF_sug + 0.10;
      if(RATIO_ON_sug - RATIO_OFF_sug < ON_gap_min) RATIO_ON_sug = RATIO_OFF_sug + ON_gap_min;
      if(RATIO_ON_sug - RATIO_OFF_sug > ON_gap_max) RATIO_ON_sug = RATIO_OFF_sug + ON_gap_max;
      if(RATIO_ON_sug > 0.95) RATIO_ON_sug = 0.95;
    }

    // Noise-based hints (relative noise at CENTER)
    double sL_center = sd(L_center)/(L_center.mean>1e-6 ? L_center.mean : 1);
    double sR_center = sd(R_center)/(R_center.mean>1e-6 ? R_center.mean : 1);
    double s_center  = (sL_center + sR_center)*0.5;
    double ERR_DEADBAND_sug = 2.5*s_center; if(ERR_DEADBAND_sug<0.010) ERR_DEADBAND_sug=0.010; if(ERR_DEADBAND_sug>0.025) ERR_DEADBAND_sug=0.025;
    double EMA_ALPHA_sug    = (s_center > 0.08)? 0.45 : 0.55;

    // ---- Output ready-to-paste ----
    banner("Suggested constants (copy to WallPD_SimpleStable_PrePID.ino)");
    Serial.print("const int   IR_L_CENTER = "); Serial.print(IR_L_CENTER_sug); Serial.println(";");
    Serial.print("const int   IR_R_CENTER = "); Serial.print(IR_R_CENTER_sug); Serial.println(";");
    Serial.print("const float  RATIO_ON    = "); Serial.print(RATIO_ON_sug,2);  Serial.println("f;");
    Serial.print("const float  RATIO_OFF   = "); Serial.print(RATIO_OFF_sug,2); Serial.println("f;");
    Serial.print("// Hints -> ERR_DEADBAND ~= "); Serial.print(ERR_DEADBAND_sug,3);
    Serial.print(" , EMA_ALPHA ~= "); Serial.println(EMA_ALPHA_sug,2);

    Serial.println("\nRaw ratios:");
    Serial.print("L_open/center=");  Serial.print(rL_open,3);
    Serial.print(" | R_open/center="); Serial.println(rR_open,3);
    Serial.print("L_close/center="); Serial.print(rL_close,3);
    Serial.print(" | R_close/center="); Serial.println(rR_close,3);

    if(openLooksBad){
      Serial.println("\nNOTE: OPEN looks too close to CENTER (r_open>0.80).");
      Serial.println("Use default OFF=0.60 / ON=0.70 or repeat with a truly empty view.");
    }

    Serial.println("\nDone.");
  }
} // namespace CAL

void setup(){
  Serial.begin(115200);          // Faster serial for clean ASCII dump
  delay(1200);                   // Give Serial Monitor time after auto-reset
  pinMode(IR_LEFT_PIN,INPUT);
  pinMode(IR_RIGHT_PIN,INPUT);
  pinMode(LED_BUILTIN,OUTPUT);

  CAL::runCalibration();
}

void loop(){ /* intentionally empty */ }