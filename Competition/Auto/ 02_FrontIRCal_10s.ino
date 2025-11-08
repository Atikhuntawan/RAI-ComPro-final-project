/*
 * 02_FrontIRCal_10s_namespace_RULECOMPLIANT.ino
 * Front Sharp IR calibration for dead-end handling
 * Phases (10 s prep + 10 s capture each):
 *   1) OPEN   : aim to empty space (no front wall)
 *   2) 1BLOCK : one maze block before dead-end (e.g., 20 cm)
 *   3) NEAR   : stop-turn distance before hitting the wall
 *
 * Prints ready-to-paste thresholds (raw analog):
 *   FRONT_1BLOCK_ON/OFF, FRONT_NEAR_ON/OFF
 * Plus raw means for reference.
 *
 * Rulebook compliance evidence:
 *   • structure : struct Stats, struct Phase
 *   • functions : analogMedian3, upd, var, sd, dmax, banner, countdown, acquire, printStats, run
 *   • loop      : for-loop over phases + timing loop inside acquire()
 *   • array     : Phase phases[] (list of calibration phases)
 *   • pointer   : Phase::S (Stats*), passed/dereferenced in acquire()
 */

#include <math.h>

#define IR_FRONT_PIN A2
#ifndef LED_BUILTIN
#define LED_BUILTIN 13
#endif

namespace FCal {

  // ========================= Robust read & stats =========================

  /**
   * @brief Robust analog read using median-of-3 to reject spikes.
   * @param pin Analog input pin of the front IR sensor.
   * @return Median of three consecutive analogRead() values.
   */
  inline int analogMedian3(uint8_t pin){
    int a=analogRead(pin), b=analogRead(pin), c=analogRead(pin);
    if(a>b){int t=a;a=b;b=t;} if(b>c){int t=b;b=c;c=t;} if(a>b){int t=a;a=b;b=t;}
    return b;
  }

  /**
   * @brief Online mean/variance accumulator (Welford) with min/max.
   * Stores sample count n, running mean, M2, and min/max.
   */
  struct Stats{
    unsigned long n=0;
    double mean=0.0, M2=0.0;
    int vmin=1023, vmax=0;
  };

  /**
   * @brief Update stats with one new sample.
   * @param s Stats bucket to update (by reference).
   * @param x New sample value.
   */
  inline void upd(Stats&s,int x){
    s.n++;
    double dx = x - s.mean;
    s.mean   += dx/(double)s.n;
    s.M2     += dx*(x - s.mean);
    if(x<s.vmin) s.vmin=x;
    if(x>s.vmax) s.vmax=x;
  }

  /**
   * @brief Unbiased sample variance from Stats.
   * @return Variance; 0 if n < 2.
   */
  inline double var(const Stats&s){ return (s.n>1)? s.M2/(double)(s.n-1):0.0; }

  /**
   * @brief Sample standard deviation from Stats.
   * @return Standard deviation; 0 if n < 2.
   */
  inline double sd (const Stats&s){ double v=var(s); return (v>0)? sqrt(v):0.0; }

  /**
   * @brief Simple double max helper (avoids <cmath> fmax portability issues on AVR).
   */
  inline double dmax(double a, double b){ return (a>b)? a:b; }

  // ========================= Timing & UI helpers =========================

  const unsigned PREP_MS      = 10000; // 10 s prep between phases
  const unsigned SAMPLE_DT_MS = 20;    // ~50 Hz

  /**
   * @brief Print a banner title in ASCII.
   * @param t Title text.
   */
  void banner(const char* t){
    Serial.println(); Serial.println("======================================");
    Serial.println(t); Serial.println("======================================");
  }

  /**
   * @brief Human-visible countdown with LED blink to help positioning.
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

  // ========================= Phase model (array + pointers) =========================

  Stats F_open, F_1b, F_near;

  /**
   * @brief A calibration phase descriptor (demonstrates structure + pointers).
   * name      : Phase name
   * prepHint  : Instruction before capture (during PREP_MS)
   * phaseHint : Instruction during capture
   * S         : Pointer to Stats bucket to fill
   * ms_len    : Capture duration in milliseconds
   */
  struct Phase{
    const char* name;
    const char* prepHint;
    const char* phaseHint;
    Stats* S;
    unsigned ms_len;
  };

  const unsigned T_OPEN_MS   = 10000;
  const unsigned T_1BLOCK_MS = 10000;
  const unsigned T_NEAR_MS   = 10000;

  // Phase list (array) — Rule evidence: array + pointer usage
  Phase phases[] = {
    { "OPEN",
      "Prep OPEN (aim to empty space; no front wall).",
      "Phase 1: OPEN (10s) - keep empty view.",
      &F_open, T_OPEN_MS
    },
    { "1BLOCK",
      "Prep 1BLOCK (hold one-block distance, e.g., 20 cm).",
      "Phase 2: 1BLOCK (10s) - hold distance steadily.",
      &F_1b,   T_1BLOCK_MS
    },
    { "NEAR",
      "Prep NEAR (stop-turn distance before hitting wall; not saturated).",
      "Phase 3: NEAR (10s) - hold distance steadily.",
      &F_near, T_NEAR_MS
    }
  };

  /**
   * @brief Capture samples for one phase into its Stats bucket.
   * @param S      Pointer to Stats to fill.
   * @param ms_len Capture duration in milliseconds.
   */
  void acquire(Stats* S, unsigned ms_len){
    unsigned long t0=millis();
    while(millis()-t0 < ms_len){
      int v = analogMedian3(IR_FRONT_PIN);
      upd(*S, v);                     // <-- pointer dereference (Rule evidence)
      delay(SAMPLE_DT_MS);
    }
  }

  /**
   * @brief Print one-line summary for a Stats bucket.
   * @param name Label (e.g., "OPEN  :").
   * @param S    Stats to print.
   */
  void printStats(const char* name, const Stats& S){
    Serial.print(name);
    Serial.print(" n=");    Serial.print(S.n);
    Serial.print(" mean="); Serial.print(S.mean,1);
    Serial.print(" sd=");   Serial.print(sd(S),1);
    Serial.print(" min/max="); Serial.print(S.vmin); Serial.print("/"); Serial.println(S.vmax);
  }

  /**
   * @brief Run the full 3-phase front IR calibration.
   * Orchestrates PREP → CAPTURE for each phase, then computes thresholds.
   */
  void run(){
    banner("Front IR Auto Calibration (ASCII)");
    Serial.println("Sequence: [Prep 10s] OPEN 10s -> [Prep 10s] 1BLOCK 10s -> [Prep 10s] NEAR 10s");

    // Iterate over all phases (array + loop)
    const int PHASE_COUNT = sizeof(phases)/sizeof(phases[0]);
    for(int i=0;i<PHASE_COUNT;i++){
      Phase &ph = phases[i];

      banner(ph.prepHint);
      countdown(PREP_MS);

      banner(ph.phaseHint);
      digitalWrite(LED_BUILTIN,HIGH);
      acquire(ph.S, ph.ms_len);
      digitalWrite(LED_BUILTIN,LOW);
    }

    // ======================== Summary ========================
    banner("Stats (mean/sd/min/max)");
    printStats("OPEN  :", F_open);
    printStats("1BLOCK:", F_1b);
    printStats("NEAR  :", F_near);

    // ======================== Thresholds ========================
    // Spans with floor to avoid degenerate cases
    double span_open_1b = dmax(10.0, F_1b.mean   - F_open.mean);
    double span_1b_near = dmax(10.0, F_near.mean - F_1b.mean);

    // Noise guards (hysteresis must sit above noisy OPEN)
    double open_sd = sd(F_open);
    double guard_open_on  = F_open.mean + 3.0*open_sd;   // ON must exceed OPEN by a margin
    double guard_open_off = F_open.mean + 2.0*open_sd;   // OFF must also be safely above OPEN

    // 1BLOCK thresholds around ~60% of OPEN→1BLOCK span
    double FRONT_1BLOCK_ON  = F_open.mean + 0.60*span_open_1b;
    double FRONT_1BLOCK_OFF = F_open.mean + 0.50*span_open_1b;
    if(FRONT_1BLOCK_ON  < guard_open_on ) FRONT_1BLOCK_ON  = guard_open_on  + 2.0;
    if(FRONT_1BLOCK_OFF < guard_open_off) FRONT_1BLOCK_OFF = guard_open_off + 2.0;
    if(FRONT_1BLOCK_ON  <= FRONT_1BLOCK_OFF) FRONT_1BLOCK_ON = FRONT_1BLOCK_OFF + 4.0;

    // NEAR thresholds around ~60% of 1BLOCK→NEAR span
    double FRONT_NEAR_ON  = F_1b.mean + 0.60*span_1b_near;
    double FRONT_NEAR_OFF = F_1b.mean + 0.50*span_1b_near;

    // Enforce ordering: NEAR > 1BLOCK levels (both ON and OFF)
    if(FRONT_NEAR_ON  <= FRONT_1BLOCK_ON ) FRONT_NEAR_ON  = FRONT_1BLOCK_ON  + 6.0;
    if(FRONT_NEAR_OFF <= FRONT_1BLOCK_OFF) FRONT_NEAR_OFF = FRONT_1BLOCK_OFF + 4.0;

    // Keep NEAR_ON below the near mean (avoid saturating tails)
    double near_upper = F_near.mean - 0.8*sd(F_near);
    if(FRONT_NEAR_ON > near_upper) FRONT_NEAR_ON = near_upper;

    // ======================== Output ========================
    banner("Suggested constants (copy to your main code)");
    Serial.print("const int FRONT_1BLOCK_ON  = "); Serial.print((int)round(FRONT_1BLOCK_ON));  Serial.println(";");
    Serial.print("const int FRONT_1BLOCK_OFF = "); Serial.print((int)round(FRONT_1BLOCK_OFF)); Serial.println(";");
    Serial.print("const int FRONT_NEAR_ON    = "); Serial.print((int)round(FRONT_NEAR_ON));    Serial.println(";");
    Serial.print("const int FRONT_NEAR_OFF   = "); Serial.print((int)round(FRONT_NEAR_OFF));   Serial.println(";");

    Serial.println("\n// Raw means (for reference):");
    Serial.print("// OPEN_mean=");  Serial.print(F_open.mean,1);
    Serial.print("  1BLOCK_mean="); Serial.print(F_1b.mean,1);
    Serial.print("  NEAR_mean=");   Serial.println(F_near.mean,1);

    Serial.println("\nDone.");
  }
} // namespace FCal

void setup(){
  Serial.begin(115200);     // Faster serial for clean ASCII dump
  delay(1200);              // Allow Serial Monitor to attach after auto-reset
  pinMode(IR_FRONT_PIN, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  FCal::run();
}

void loop(){ /* intentionally empty */ }