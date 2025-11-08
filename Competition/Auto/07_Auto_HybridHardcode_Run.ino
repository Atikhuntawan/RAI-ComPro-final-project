/*
 * Auto_HybridHardcode_Run.ino
 *
 * Hybrid Hardcoding for the given maze layout:
 * 1) From START (bottom-left) → go UP to OBJ → PICK
 * 2) Continue UP to top-left lane → TURN RIGHT → go along top to TARGET → DROP
 * 3) TURN 180° → come back along top to EXIT → WALLSET & STOP
 *
 * Modules included:
 *  - Wall PD centering (IR L/R)              [median-of-3 + EMA + hysteresis]
 *  - Gyro PD turn (MPU6050 GyroZ)            [KP_ANG*err - KD_VEL*vel]
 *  - Front IR thresholds (1BLOCK / NEAR)     [dead-end handling / wallset]
 *  - Gripper (rotate+claw servos) + switch
 *
 * Rulebook evidence: function + struct + loop + array + pointer
 * (ทุกฟังก์ชันมีคอมเมนต์อังกฤษบอกหน้าที่ชัดเจน)
 */

#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <math.h>

/* ============================== Motor pins ============================== */
#define ENA 5
#define IN1 2
#define IN2 4
#define ENB 6
#define IN3 7
#define IN4 8

/* ============================== IR pins ================================= */
#define IR_LEFT_PIN    A0
#define IR_RIGHT_PIN   A1
#define IR_FRONT_PIN   A2

/* ============================== Servo & Switch =========================== */
#define PIN_SERVO_ROTATE 9
#define PIN_SERVO_CLAW   10
#define PIN_INSCOOP_SW   3   // SS-5GL: COM->GND, NO->D3, use INPUT_PULLUP

/* ============================== Tunables (paste your latest calib!) ===== */
// --- Side IR centers (from your calibration sketch) ---
const int   IR_L_CENTER = 271;     // <-- paste latest
const int   IR_R_CENTER = 268;     // <-- paste latest
const float RATIO_ON    = 0.70f;   // wall becomes present
const float RATIO_OFF   = 0.60f;   // wall becomes absent

// --- Filters for side IR ---
const float ERR_DEADBAND = 0.025f; // relative deadband on error
const float EMA_ALPHA    = 0.55f;

// --- Front IR thresholds (from FrontIRCal) ---
const int FRONT_1BLOCK_ON  = 530;  // <-- paste measured
const int FRONT_1BLOCK_OFF = 500;
const int FRONT_NEAR_ON    = 650;  // stop/turn before wall
const int FRONT_NEAR_OFF   = 610;

// --- Straight run speed & timing (tune in field) ---
int   BASE_L = 105, BASE_R = 86;   // base PWM per side
int   TRIM_L = 0,   TRIM_R = 0;    // small hardware bias trims
const int MIN_PWM_L = 66, MIN_PWM_R = 60;
const unsigned MS_PER_BLOCK = 850; // time for one 20x20 cm block at base speed (tune!)

/* ============================== PD gains (side) ========================= */
float Kp_side = 205.0f;
float Kd_side = 65.0f;
const int   CORR_MAX = 130;
const int   CORR_MIN_EFFECT = 15;
const float SPEED_SLOPE = 0.35f;
const float MIN_BASE_SCALE = 0.85f;
const float CORR_SLEW_PER_S = 2400.0f;

/* ============================== Turn PD (gyro) ========================== */
const unsigned TURN_TIMEOUT_MS = 3500;
const float KP_ANG   = 3.6f;
const float KD_VEL   = 0.90f;
const int   PWM_MIN  = 70;
const int   PWM_MAX  = 120;
const float DONE_ERR = 1.0f;
const float DONE_VEL = 5.0f;
const unsigned LOOP_MS = 12;
const unsigned BRAKE_L_MS = 35;
const unsigned BRAKE_R_MS = 35;
const int BOOST_PIVOT_R_LFWD = 16;
const int BOOST_PIVOT_L_RFWD = 14;

const uint8_t MPU_ADDR = 0x68;
const float   GYRO_SCALE = 131.0f;
const float   VEL_DEADBAND = 0.35f;
const float   LPF_ALPHA  = 0.22f;

/* ============================== Gripper angles ========================== */
struct Angles { int rotateDown, rotateUp, clawOpen, clawGrip, clawHold; };
Angles ANG{ 10, 110, 30, 120, 60 };

struct GripTune {
  int rotStepDeg, rotStepDelay, rotPauseStage;
  int wiggleAmpl, wiggleCycles, wiggleDelay;
  int gripRampTotal, gripRampStep, retryMax;
  int boostOvershoot, boostHoldMs, debounceMs;
};
GripTune TUNE{ 2,15,350, 3,4,60, 8,2,2, 4,120,12 };

/* ============================== Small utils ============================= */
static inline int clampi(int v,int lo,int hi){ return v<lo?lo:(v>hi?hi:v); }
static inline int applyMinL(int v){ return (v>0 && v<MIN_PWM_L)? MIN_PWM_L : v; }
static inline int applyMinR(int v){ return (v>0 && v<MIN_PWM_R)? MIN_PWM_R : v; }
int analogMedian3(uint8_t pin){
  int a=analogRead(pin), b=analogRead(pin), c=analogRead(pin);
  if(a>b){int t=a;a=b;b=t;} if(b>c){int t=b;b=c;c=t;} if(a>b){int t=a;a=b;b=t;}
  return b;
}

/* ============================== Side IR filtering ======================= */
struct IRFiltered { int L=0,R=0, emaL=0, emaR=0; };
struct WallFlags { bool left=false, right=false; };
IRFiltered irf;
WallFlags  walls;

/** Read & EMA-filter side IR values. */
void readSideIR(IRFiltered* o){
  int rl=analogMedian3(IR_LEFT_PIN);
  int rr=analogMedian3(IR_RIGHT_PIN);
  if(o->emaL==0 && o->emaR==0){ o->emaL=rl; o->emaR=rr; }
  o->emaL=(int)(EMA_ALPHA*rl + (1.0f-EMA_ALPHA)*o->emaL);
  o->emaR=(int)(EMA_ALPHA*rr + (1.0f-EMA_ALPHA)*o->emaR);
  o->L=o->emaL; o->R=o->emaR;
}
/** Update wall presence hysteresis. */
void updateWallFlags(const IRFiltered* s, WallFlags* wf){
  float lratio=(float)s->L/(float)IR_L_CENTER;
  float rratio=(float)s->R/(float)IR_R_CENTER;
  if(!wf->left  && lratio>=RATIO_ON ) wf->left  = true;
  if( wf->left  && lratio<=RATIO_OFF) wf->left  = false;
  if(!wf->right && rratio>=RATIO_ON ) wf->right = true;
  if( wf->right && rratio<=RATIO_OFF) wf->right = false;
}
/** Compute lateral error for PD (two/single/no walls). */
float computeSideError(const IRFiltered* s, const WallFlags* wf){
  float lratio=(float)s->L/(float)IR_L_CENTER;
  float rratio=(float)s->R/(float)IR_R_CENTER;
  float e=0.0f;
  if(wf->left && wf->right)       e = lratio - rratio;
  else if(wf->left && !wf->right) e = lratio - 1.0f;
  else if(!wf->left && wf->right) e = 1.0f - rratio;
  else                            e = 0.0f;
  if(e>-ERR_DEADBAND && e<ERR_DEADBAND) e=0.0f;
  return e;
}

/* ============================== Motor layer ============================= */
void motorLeft(int s){
  if (s > 0){ digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); analogWrite(ENA, s); }
  else if (s < 0){ digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); analogWrite(ENA, -s); }
  else { analogWrite(ENA,0); digitalWrite(IN1,LOW); digitalWrite(IN2,LOW); }
}
void motorRight(int s){
  if (s > 0){ digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);  analogWrite(ENB, s); }
  else if (s < 0){ digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); analogWrite(ENB, -s); }
  else { analogWrite(ENB,0); digitalWrite(IN3,LOW); digitalWrite(IN4,LOW); }
}
void motorStop(){ motorLeft(0); motorRight(0); }
void motorForwardPWM(int pwmL,int pwmR){
  pwmL = applyMinL(clampi(pwmL,0,255));
  pwmR = applyMinR(clampi(pwmR,0,255));
  motorLeft(+pwmL);
  motorRight(+pwmR);
}

/* ============================== Front IR state ========================== */
enum FrontState { F_OPEN=0, F_1BLOCK=1, F_NEAR=2 };
FrontState frontSense(){
  int v = analogMedian3(IR_FRONT_PIN);
  static bool one=false, near=false;
  if(!one && v>=FRONT_1BLOCK_ON) one=true;
  if( one && v<=FRONT_1BLOCK_OFF)one=false;
  if(!near&& v>=FRONT_NEAR_ON)   near=true;
  if( near&& v<=FRONT_NEAR_OFF)  near=false;
  if(near) return F_NEAR;
  if(one)  return F_1BLOCK;
  return F_OPEN;
}

/* ============================== Wall-PD straight drive ================== */
/** Run wall-PD for given milliseconds (emergency stop if front NEAR). */
void driveStraight_wallPD(unsigned ms){
  unsigned long t0=millis();
  float e_prev=0.0f;
  int corr_prev=0;

  while(millis()-t0 < ms){
    readSideIR(&irf);
    updateWallFlags(&irf,&walls);
    float e = computeSideError(&irf,&walls);
    float dt = LOOP_MS/1000.0f;
    float d = (e - e_prev)/dt;
    float u = Kp_side*e + Kd_side*d;

    if(u >  CORR_MAX) u =  CORR_MAX;
    if(u < -CORR_MAX) u = -CORR_MAX;
    int corrTarget = (int)u;
    if(corrTarget>0 && corrTarget<CORR_MIN_EFFECT)  corrTarget =  CORR_MIN_EFFECT;
    if(corrTarget<0 && -corrTarget<CORR_MIN_EFFECT) corrTarget = -CORR_MIN_EFFECT;

    int maxStep = (int)(CORR_SLEW_PER_S * dt); if(maxStep<1) maxStep=1;
    int diff = corrTarget - corr_prev;
    if(diff >  maxStep) diff =  maxStep;
    if(diff < -maxStep) diff = -maxStep;
    int corr = corr_prev + diff;
    corr_prev = corr;

    float baseScale = 1.0f - SPEED_SLOPE * fabs(e);
    if (baseScale < MIN_BASE_SCALE) baseScale = MIN_BASE_SCALE;

    int baseL = (int)(BASE_L * baseScale) + TRIM_L;
    int baseR = (int)(BASE_R * baseScale) + TRIM_R;

    int pwmL = clampi(baseL + corr, 0, 255);
    int pwmR = clampi(baseR - corr, 0, 255);

    if(!walls.left && !walls.right){ pwmL=baseL; pwmR=baseR; } // safe fallback

    motorForwardPWM(pwmL,pwmR);

    // Emergency stop if nearly hits a wall in front
    if(frontSense()==F_NEAR) break;

    e_prev = e;
    delay(LOOP_MS);
  }
  motorStop();
}

/* ============================== Gyro turn PD ============================ */
struct IMUState { float gyroZ_bias=-0.22f, velZ_dps_f=0.0f, angleZ_deg=0.0f; unsigned long lastUs=0; } imu;

void mpuWrite(uint8_t reg, uint8_t val){
  Wire.beginTransmission(MPU_ADDR); Wire.write(reg); Wire.write(val); Wire.endTransmission(true);
}
void mpuRead(uint8_t reg, uint8_t n, uint8_t *buf){
  Wire.beginTransmission(MPU_ADDR); Wire.write(reg); Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, n, (uint8_t)true);
  for(uint8_t i=0;i<n && Wire.available();++i) buf[i] = Wire.read();
}
void mpuInit(){
  mpuWrite(0x6B, 0x00); // wake
  mpuWrite(0x1A, 0x03); // DLPF ~44Hz
  mpuWrite(0x1B, 0x00); // ±250 dps
  delay(50);
}
float readGyroZ_dps_raw(){
  uint8_t b[2]; mpuRead(0x47,2,b);
  int16_t z = (int16_t)((b[0]<<8)|b[1]);
  return (float)z / GYRO_SCALE;
}
void calibrateGyroZ(IMUState* s, unsigned samples=800){
  double sum=0; delay(20);
  for(unsigned i=0;i<samples;i++){ sum += readGyroZ_dps_raw(); delay(1); }
  s->gyroZ_bias = (float)(sum / (double)samples);
}
void resetAngle(IMUState* s){ s->angleZ_deg=0.0f; s->lastUs=micros(); }
void updateAngle(IMUState* s){
  unsigned long now=micros();
  float dt=(now - s->lastUs)*1e-6f; s->lastUs=now;
  float vel = readGyroZ_dps_raw() - s->gyroZ_bias;
  if(fabs(vel) < VEL_DEADBAND) vel = 0.0f;
  s->velZ_dps_f += LPF_ALPHA * (vel - s->velZ_dps_f);
  s->angleZ_deg += s->velZ_dps_f * dt;
}
float getAngle(const IMUState* s){ return s->angleZ_deg; }

void motorBrakeShort(unsigned ms=30){
  analogWrite(ENA,255); digitalWrite(IN1,HIGH); digitalWrite(IN2,HIGH);
  analogWrite(ENB,255); digitalWrite(IN3,HIGH); digitalWrite(IN4,HIGH);
  delay(ms); motorStop();
}
void spinPivotBiased(int dir, int pwm){
  int pL=pwm, pR=pwm;
  if (dir>0)  pR += BOOST_PIVOT_L_RFWD;
  else        pL += BOOST_PIVOT_R_LFWD;
  if (dir>0){ motorLeft(-pL); motorRight(+pR); }
  else      { motorLeft(+pL); motorRight(-pR); }
}
void trackSettle(IMUState* s, unsigned ms=140){
  unsigned long t0=millis();
  while(millis()-t0 < ms){ updateAngle(s); delay(2); }
}
/** Turn to target angle (+ left / − right) with PD; returns final angle. */
float turnToAnglePD(IMUState* s, float targetDeg){
  resetAngle(s);
  unsigned long t0=millis(); int pwmPrev=0;
  while(millis()-t0 < TURN_TIMEOUT_MS){
    updateAngle(s);
    float ang=getAngle(s), err=targetDeg - ang, vel=s->velZ_dps_f;
    if (fabs(err)<=DONE_ERR && fabs(vel)<=DONE_VEL){
      motorBrakeShort(targetDeg>0?BRAKE_L_MS:BRAKE_R_MS); break;
    }
    float u = KP_ANG*err - KD_VEL*vel;
    int sgn=(u>=0.0f)?+1:-1;
    int pwm=(int)fabs(u);
    int maxNear = map((int)constrain((int)fabs(err),0,20), 0,20, PWM_MIN+2, PWM_MAX);
    if(pwm<PWM_MIN) pwm=PWM_MIN; if(pwm>maxNear) pwm=maxNear;
    const int PWM_RAMP_UP=6; if(pwm>pwmPrev+PWM_RAMP_UP) pwm=pwmPrev+PWM_RAMP_UP; pwmPrev=pwm;
    spinPivotBiased(sgn,pwm);
    delay(LOOP_MS);
  }
  motorStop(); trackSettle(s,140); return getAngle(s);
}

/* ============================== Gripper (servo) ======================== */
Servo sRotate, sClaw;
bool readSwitchDebounced(uint8_t pin, uint16_t ms){
  static uint32_t t=0; static int last=HIGH, stable=HIGH;
  int v=digitalRead(pin); if(v!=last){last=v; t=millis();}
  if(millis()-t>ms) stable=v; return (stable==LOW);
}
void rampRotate(int fromDeg,int toDeg,const GripTune* tn){
  int step=(toDeg>=fromDeg)?+tn->rotStepDeg:-tn->rotStepDeg;
  for(int a=fromDeg; a!=toDeg; a+=step){
    sRotate.write(a); delay(tn->rotStepDelay);
    if(abs(toDeg-a)<abs(step)) break;
  } sRotate.write(toDeg);
}
void wiggleRotate(int center,const GripTune* tn){
  for(int i=0;i<tn->wiggleCycles;i++){
    sRotate.write(center + tn->wiggleAmpl); delay(tn->wiggleDelay);
    sRotate.write(center - tn->wiggleAmpl); delay(tn->wiggleDelay);
  } sRotate.write(center);
}
/** Assisted lift (2-stage ramp + grip ramp + tiny overshoot). */
bool liftWithAssist(const Angles* ang,const GripTune* tn){
  wiggleRotate(ang->rotateDown, tn);
  int targetGrip=ang->clawGrip + tn->gripRampTotal;
  int curGrip=ang->clawGrip; sClaw.write(curGrip);
  int stage1Target=min(ang->rotateDown+25, ang->rotateUp);
  for(int a=ang->rotateDown; a<=stage1Target; a+=tn->rotStepDeg){
    sRotate.write(a);
    if(curGrip<targetGrip && ((a-ang->rotateDown)%(tn->rotStepDeg*2)==0)){
      curGrip=min(curGrip+tn->gripRampStep, targetGrip); sClaw.write(curGrip);
    }
    delay(tn->rotStepDelay);
  }
  delay(tn->rotPauseStage);
  int boostTo=min(stage1Target + tn->boostOvershoot, ang->rotateUp);
  sRotate.write(boostTo); delay(tn->boostHoldMs);
  sRotate.write(stage1Target); delay(80);
  rampRotate(stage1Target, ang->rotateUp, tn);
  return true;
}

/* ============================== High-level actions ===================== */
/** Approach forward slowly until FRONT_NEAR, brake+square, then stop. */
void wallSet(){
  unsigned long t0=millis();
  while(millis()-t0 < 3000){
    // gentle slow approach with small base
    int baseL = clampi((int)(BASE_L*0.62f)+TRIM_L, 0, 255);
    int baseR = clampi((int)(BASE_R*0.62f)+TRIM_R, 0, 255);
    motorForwardPWM(baseL, baseR);
    if(frontSense()==F_NEAR){ motorBrakeShort(35); break; }
    delay(15);
  }
  motorStop();
}
/** Drive N blocks using wall PD (safety stop if FRONT_NEAR). */
void driveBlocks(uint8_t n){
  for(uint8_t i=0;i<n;i++){
    driveStraight_wallPD(MS_PER_BLOCK);
  }
}
/** PICK sequence at OBJ: open → rotate down → wait switch → grip → lift. */
void actionPick(){
  // ensure claw open and arm down
  sClaw.write(ANG.clawOpen); delay(250);
  rampRotate(ANG.rotateDown+30, ANG.rotateDown, &TUNE);
  // creep to wall and wait switch contact (if used)
  wallSet();
  // grip then lift
  sClaw.write(ANG.clawGrip); delay(250);
  liftWithAssist(&ANG,&TUNE);
}
/** DROP sequence at TARGET: rotate down and open; then lift up again. */
void actionDrop(){
  rampRotate(ANG.rotateUp, ANG.rotateDown+8, &TUNE);
  sClaw.write(ANG.clawOpen); delay(250);
  rampRotate(ANG.rotateDown+8, ANG.rotateUp, &TUNE);
}

/* ============================== Script (plan) ========================== */
/* Tune these counts to match the pictured layout exactly */
#define BLKS_UP_TO_OBJ         3   // START → OBJ (UP)
#define BLKS_UP_TO_TOP         2   // OBJ → reach top-left lane (UP)
#define BLKS_TOP_TO_TARGET     7   // top-left → TARGET (RIGHT along top)
#define BLKS_TOP_BACK_TO_EXIT  7   // TARGET → back to EXIT (LEFT along top)

enum StepType : uint8_t {
  DRIVE_BLKS=0, TURN_L90, TURN_R90, TURN_180, WALLSET_STEP,
  PICK_OBJ, DROP_OBJ, WAIT_MS, END_SCRIPT
};
struct Step { StepType type; int value; };

Step plan[] = {
  {DRIVE_BLKS, BLKS_UP_TO_OBJ},     // go UP to OBJ
  {WALLSET_STEP, 0},
  {PICK_OBJ, 0},

  {DRIVE_BLKS, BLKS_UP_TO_TOP},     // continue UP to top-left lane
  {TURN_R90, 0},                    // face RIGHT along top row

  {DRIVE_BLKS, BLKS_TOP_TO_TARGET},
  {WALLSET_STEP, 0},
  {DROP_OBJ, 0},                    // deliver at TARGET

  {TURN_L90, 0}, {TURN_L90, 0},     // 180° turn
  {DRIVE_BLKS, BLKS_TOP_BACK_TO_EXIT},
  {WALLSET_STEP, 0},                // square at EXIT
  {END_SCRIPT, 0}
};

/* ============================== Arduino setup / loop =================== */
void setup(){
  Serial.begin(115200);
  // Motor pins
  pinMode(ENA,OUTPUT); pinMode(IN1,OUTPUT); pinMode(IN2,OUTPUT);
  pinMode(ENB,OUTPUT); pinMode(IN3,OUTPUT); pinMode(IN4,OUTPUT);
  motorStop();

  // IR pins
  pinMode(IR_LEFT_PIN,INPUT); pinMode(IR_RIGHT_PIN,INPUT);
  pinMode(IR_FRONT_PIN,INPUT);

  // Servos & switch
  pinMode(PIN_INSCOOP_SW, INPUT_PULLUP);
  sRotate.attach(PIN_SERVO_ROTATE);
  sClaw.attach(PIN_SERVO_CLAW);
  // Initial posture
  sClaw.write(ANG.clawOpen); delay(400);
  int preTouch = ANG.rotateDown + 30; sRotate.write(preTouch); delay(250);
  rampRotate(preTouch, ANG.rotateDown, &TUNE);

  // IMU
  Wire.begin(); mpuInit();
  Serial.println(F("# Calibrating gyroZ... keep still"));
  calibrateGyroZ(&imu, 800);
  Serial.print(F("# gyroZ_bias = ")); Serial.println(imu.gyroZ_bias,5);
  Serial.println(F("# Auto plan start"));
}

void loop(){
  static bool started=false;
  if(!started){ delay(1000); started=true; }

  // Execute plan
  for(size_t i=0; i<sizeof(plan)/sizeof(plan[0]); ++i){
    Step* st = &plan[i];
    switch(st->type){
      case DRIVE_BLKS:      driveBlocks((uint8_t)st->value);         break;
      case TURN_L90:        turnToAnglePD(&imu, +90.0f);             break;
      case TURN_R90:        turnToAnglePD(&imu, -90.0f);             break;
      case TURN_180:        turnToAnglePD(&imu, 180.0f);             break;
      case WALLSET_STEP:    wallSet();                                break;
      case PICK_OBJ:        actionPick();                             break;
      case DROP_OBJ:        actionDrop();                             break;
      case WAIT_MS:         delay((unsigned)st->value);               break;
      case END_SCRIPT:      motorStop(); while(true){ delay(1000); }  break;
    }
    delay(120); // small gap between steps
  }

  // Safety: if plan ever ends without END_SCRIPT, stop here
  motorStop(); while(true){ delay(1000); }
}