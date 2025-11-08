/*
 * WallPD_SimpleStable_PrePID_RULECOMPLIANT.ino
 *
 * Purpose:
 *   Side-wall PD centering with two Sharp IR sensors (L/R).
 *   - Start with a short open-loop warmup (no PID), then enable PD.
 *   - Supports two walls or single-side wall following.
 *   - Signal hygiene: median-of-3 + EMA + deadband + hysteresis.
 *
 * Rulebook compliance evidence:
 *   • structure : struct IRFiltered, struct WallFlags, struct PDState
 *   • functions : analogMedian3, readIR, updateWallFlags, computeError,
 *                 applySlew, computeBasePWM, motorWrite
 *   • loop      : main control loop in loop()
 *   • array     : IR_CENTER[2] (demonstrates array use)
 *   • pointer   : readIR(IRFiltered*), updateWallFlags(const IRFiltered*, WallFlags*),
 *                 computeBasePWM(float,int*,int*) use pointers
 */

#include <Arduino.h>
#include <math.h>

/* ======================= Motor pins & directions ======================= */
#define A_EN 5
#define A_IN1 2
#define A_IN2 4
#define B_EN 6
#define B_IN3 7
#define B_IN4 8

#define LEFT_IS_CHANNEL_A 0   // 0: left=B, right=A  | 1: left=A, right=B
#if LEFT_IS_CHANNEL_A
  #define LEFT_EN     A_EN
  #define LEFT_FWD()  do{ digitalWrite(A_IN1,HIGH); digitalWrite(A_IN2,LOW); }while(0)
  #define RIGHT_EN    B_EN
  #define RIGHT_FWD() do{ digitalWrite(B_IN3,LOW);  digitalWrite(B_IN4,HIGH);}while(0)
#else
  #define LEFT_EN     B_EN
  #define LEFT_FWD()  do{ digitalWrite(B_IN3,LOW);  digitalWrite(B_IN4,HIGH);}while(0)
  #define RIGHT_EN    A_EN
  #define RIGHT_FWD() do{ digitalWrite(A_IN1,HIGH); digitalWrite(A_IN2,LOW); }while(0)
#endif

/* ======================= IR pins & calibration ======================== */
#define IR_LEFT_PIN   A0
#define IR_RIGHT_PIN  A1

// Paste your latest calibrated values here (from your calibration sketches)
const int   IR_L_CENTER = 263;
const int   IR_R_CENTER = 292;

// Thresholds for wall presence hysteresis (ratio to CENTER)
const float RATIO_ON  = 0.67f;  // rising: becomes "has wall"
const float RATIO_OFF = 0.57f;  // falling: becomes "no wall"

// Filter/deadband (guided by the calibration hints)
const float ERR_DEADBAND = 0.025f;
const float EMA_ALPHA    = 0.55f;

/* Demonstrate "array" usage: CENTER reference per side */
enum Side { LEFT=0, RIGHT=1 };
const int IR_CENTER[2] = { IR_L_CENTER, IR_R_CENTER };

/* ======================= Drive base & trims =========================== */
int BASE_L = 105, BASE_R = 86;    // adjust during warmup to match straight run
int TRIM_L = 0,   TRIM_R = 0;     // compensate mechanical bias if any
const int MIN_PWM_L = 66, MIN_PWM_R = 60;

/* ======================= PD control knobs ============================= */
float Kp = 205.0f;
float Kd = 65.0f;
const int   CORR_MAX = 130;
const int   CORR_MIN_EFFECT = 15;
const float SPEED_SLOPE = 0.35f;
const float MIN_BASE_SCALE = 0.85f;
const float CORR_SLEW_PER_S = 2400.0f;

/* ======================= Warmup (open-loop) =========================== */
const unsigned WARMUP_MS = 500;        // ~0.5 s warmup before enabling PD
const int      START_KICK_PWM = 120;   // initial kick to overcome stiction
const unsigned START_KICK_MS  = 150;

/* ======================= Controller state (structures) ================ */
/**
 * @brief Holds filtered IR readings and EMA states.
 *  L/R : current filtered values; emaL/emaR : internal EMA accumulators.
 */
struct IRFiltered { int L=0, R=0, emaL=0, emaR=0; };

/**
 * @brief L/R wall presence flags after hysteresis decision.
 */
struct WallFlags { bool left=false, right=false; };

/**
 * @brief PD controller & run-time state.
 *  t_start    : start timestamp for warmup & kick
 *  pidEnabled : flag indicating PD mode entered
 *  e_prev     : previous lateral error (for derivative term)
 *  corr_prev  : previous correction (for slew-rate limiting)
 *  t_prev_ms  : previous loop timestamp (for dt)
 */
struct PDState {
  unsigned long t_start=0;
  bool  pidEnabled=false;
  float e_prev=0.0f;
  int   corr_prev=0;
  unsigned long t_prev_ms=0;
} ctrl;

/* ======================= Small utilities (functions) ================== */
/**
 * @brief Clamp integer v to [lo, hi].
 */
static inline int clampi(int v,int lo,int hi){ return v<lo?lo:(v>hi?hi:v); }

/**
 * @brief Enforce side-specific minimum PWM if positive and below minimum.
 */
static inline int applyMinL(int v){ return (v>0 && v<MIN_PWM_L)? MIN_PWM_L : v; }
static inline int applyMinR(int v){ return (v>0 && v<MIN_PWM_R)? MIN_PWM_R : v; }

/**
 * @brief Robust analog read: median-of-3 to reject spikes.
 * @param pin  Analog input pin to read from.
 * @return Median of three consecutive analogRead() calls.
 */
int analogMedian3(uint8_t pin){
  int a=analogRead(pin), b=analogRead(pin), c=analogRead(pin);
  if(a>b){int t=a;a=b;b=t;} if(b>c){int t=b;b=c;c=t;} if(a>b){int t=a;a=b;b=t;}
  return b;
}

/**
 * @brief Read and filter IR sensors into IRFiltered (EMA over median-of-3).
 * @param out Pointer to IRFiltered to write (uses and updates out->emaL/R).
 */
void readIR(IRFiltered* out){
  int rl = analogMedian3(IR_LEFT_PIN);
  int rr = analogMedian3(IR_RIGHT_PIN);
  if(out->emaL==0 && out->emaR==0){ out->emaL=rl; out->emaR=rr; } // bootstrap
  out->emaL = (int)(EMA_ALPHA*rl + (1.0f-EMA_ALPHA)*out->emaL);
  out->emaR = (int)(EMA_ALPHA*rr + (1.0f-EMA_ALPHA)*out->emaR);
  out->L = out->emaL;
  out->R = out->emaR;
}

/**
 * @brief Update wall presence flags with hysteresis using ratios to CENTER.
 * @param ir   Filtered readings (L/R).
 * @param wf   Wall flags to update (left/right).
 */
void updateWallFlags(const IRFiltered* ir, WallFlags* wf){
  float lratio = (float)ir->L / (float)IR_CENTER[LEFT];
  float rratio = (float)ir->R / (float)IR_CENTER[RIGHT];

  if(!wf->left  && lratio>=RATIO_ON ) wf->left  = true;
  if( wf->left  && lratio<=RATIO_OFF) wf->left  = false;
  if(!wf->right && rratio>=RATIO_ON ) wf->right = true;
  if( wf->right && rratio<=RATIO_OFF) wf->right = false;
}

/**
 * @brief Compute lateral error for PD:
 *   - two walls : e = lratio - rratio (center between walls)
 *   - left only : e = lratio - 1
 *   - right only: e = 1 - rratio
 *   - none      : e = 0
 * Deadband is applied to suppress tiny oscillations.
 * @return Signed error (left positive).
 */
float computeError(const IRFiltered* ir, const WallFlags* wf){
  float lratio = (float)ir->L / (float)IR_CENTER[LEFT];
  float rratio = (float)ir->R / (float)IR_CENTER[RIGHT];

  float e=0.0f;
  if(wf->left && wf->right)       e = lratio - rratio;
  else if(wf->left && !wf->right) e = lratio - 1.0f;
  else if(!wf->left && wf->right) e = 1.0f - rratio;
  else                            e = 0.0f; // no walls → do not correct

  if(e>-ERR_DEADBAND && e<ERR_DEADBAND) e=0.0f;
  return e;
}

/**
 * @brief Slew-rate limiter for correction term (prevents sudden jerks).
 * @param target Desired correction target (corrTarget).
 * @param prev   Previous correction (stored in ctrl.corr_prev).
 * @param dt     Loop delta time in seconds.
 * @return New correction after applying rate limit.
 */
int applySlew(int target, int prev, float dt){
  int maxStep = (int)(CORR_SLEW_PER_S * dt);
  if(maxStep<1) maxStep=1;
  int diff = target - prev;
  if(diff >  maxStep) diff =  maxStep;
  if(diff < -maxStep) diff = -maxStep;
  return prev + diff;
}

/**
 * @brief Compute base PWM (after speed scaling and trims).
 * @param baseScale Speed scale factor [MIN_BASE_SCALE..1].
 * @param outL/outR Output base PWM for L/R (via pointers).
 */
void computeBasePWM(float baseScale, int* outL, int* outR){
  int bL = (int)(BASE_L * baseScale) + TRIM_L;
  int bR = (int)(BASE_R * baseScale) + TRIM_R;
  if(outL) *outL = bL;
  if(outR) *outR = bR;
}

/**
 * @brief Write motor PWMs with forward direction. Applies:
 *   - start-kick for the first START_KICK_MS after power-up
 *   - min-PWM per side (if positive)
 */
void motorWrite(int pwmL,int pwmR){
  LEFT_FWD(); RIGHT_FWD();

  // initial kick to overcome static friction
  if(millis()-ctrl.t_start < START_KICK_MS){
    if(pwmL<START_KICK_PWM) pwmL=START_KICK_PWM;
    if(pwmR<START_KICK_PWM) pwmR=START_KICK_PWM;
  }

  pwmL = applyMinL(clampi(pwmL,0,255));
  pwmR = applyMinR(clampi(pwmR,0,255));
  analogWrite(LEFT_EN,  pwmL);
  analogWrite(RIGHT_EN, pwmR);
}

/* ======================= Arduino setup/loop =========================== */
void setup(){
  pinMode(A_IN1,OUTPUT); pinMode(A_IN2,OUTPUT);
  pinMode(B_IN3,OUTPUT); pinMode(B_IN4,OUTPUT);
  pinMode(A_EN,OUTPUT);  pinMode(B_EN,OUTPUT);
  pinMode(IR_LEFT_PIN,INPUT); pinMode(IR_RIGHT_PIN,INPUT);
  Serial.begin(115200);

  delay(3000);               // get ready
  ctrl.t_start  = millis();
  ctrl.t_prev_ms= millis();
  ctrl.pidEnabled=false;
  ctrl.e_prev   = 0.0f;
  ctrl.corr_prev= 0;
}

void loop(){
  unsigned long t_now_ms = millis();
  float dt = (t_now_ms - ctrl.t_prev_ms)/1000.0f; if(dt<=0) dt=0.001f;
  ctrl.t_prev_ms = t_now_ms;

  // Enable PD after warmup
  if(!ctrl.pidEnabled && (t_now_ms - ctrl.t_start) >= WARMUP_MS){
    ctrl.pidEnabled = true;
    ctrl.corr_prev  = 0;       // avoid jerk when switching modes
    ctrl.e_prev     = 0.0f;
  }

  IRFiltered ir; readIR(&ir);  // filtered IR for stable decisions
  WallFlags wf; updateWallFlags(&ir, &wf);

  if(!ctrl.pidEnabled){
    // ========== Warmup: open-loop straight run ==========
    int baseL, baseR; computeBasePWM(/*baseScale=*/1.0f, &baseL, &baseR);
    motorWrite(baseL, baseR);

    // Debug (slow)
    static unsigned long last=0; if(t_now_ms-last>120){
      Serial.print("WARMUP L:");Serial.print(ir.L);
      Serial.print(" R:");Serial.print(ir.R);
      Serial.print(" pwmL/R:");Serial.print(baseL);Serial.print("/");Serial.print(baseR);
      Serial.println();
      last=t_now_ms;
    }
    return;
  }

  // ========== PD mode ==========
  float e = computeError(&ir, &wf);
  float d = (e - ctrl.e_prev)/dt;
  float u = Kp*e + Kd*d;

  // saturate and enforce minimum effective correction
  if(u >  CORR_MAX) u =  CORR_MAX;
  if(u < -CORR_MAX) u = -CORR_MAX;
  int corrTarget = (int)u;
  if(corrTarget>0 && corrTarget<CORR_MIN_EFFECT)  corrTarget =  CORR_MIN_EFFECT;
  if(corrTarget<0 && -corrTarget<CORR_MIN_EFFECT) corrTarget = -CORR_MIN_EFFECT;

  // slew-limit correction
  int corr = applySlew(corrTarget, ctrl.corr_prev, dt);
  ctrl.corr_prev = corr;

  // speed scaling with error magnitude (prevent overshoot)
  float baseScale = 1.0f - SPEED_SLOPE * fabsf(e);
  if(baseScale < MIN_BASE_SCALE) baseScale = MIN_BASE_SCALE;

  int baseL, baseR; computeBasePWM(baseScale, &baseL, &baseR);
  int pwmL = clampi(baseL + corr, 0, 255);
  int pwmR = clampi(baseR - corr, 0, 255);

  // If no walls detected, revert to base straight run (safe fallback)
  if(!wf.left && !wf.right){ pwmL = baseL; pwmR = baseR; }

  motorWrite(pwmL, pwmR);
  ctrl.e_prev = e;

  // Debug (slow)
  static unsigned long last=0; if(t_now_ms-last>100){
    Serial.print("PID  L:");Serial.print(ir.L);
    Serial.print(" R:");Serial.print(ir.R);
    Serial.print(" e:");Serial.print(e,3);
    Serial.print(" corr:");Serial.print(corr);
    Serial.print(" wall(L,R):");Serial.print(wf.left);Serial.print(",");Serial.print(wf.right);
    Serial.print(" pwmL/R:");Serial.print(pwmL);Serial.print("/");Serial.println(pwmR);
    last=t_now_ms;
  }
}