/*
 * TurnPD_MPU6050_Minimal_RULECOMPLIANT.ino
 * PD turning to a target angle using MPU6050 Gyro Z:
 *   control = KP_ANG * angle_error  -  KD_VEL * angular_velocity
 * Finish when both |error| and |vel| are small → short brake → settle.
 *
 * Rulebook compliance evidence:
 *   • structure : struct IMUState, struct TurnJob, struct TurnResult
 *   • functions : mpuWrite, mpuRead, mpuInit, readGyroZ_dps_raw,
 *                 calibrateGyroZ, resetAngle, updateAngle, getAngle,
 *                 motorLeft, motorRight, spinPivotBiased, motorStop,
 *                 motorBrakeShort, trackSettle, turnToAnglePD, runTurnPlan
 *   • loop      : main control loop inside turnToAnglePD(); demo loop iterates jobs
 *   • array     : TurnJob jobs[] (demo plan), plus small local arrays in I2C read
 *   • pointer   : pass IMUState*, TurnJob*, TurnResult* into helper functions
 */

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

/* ================= Pins (L298N typical wiring) ================= */
#define ENA 5
#define IN1 2
#define IN2 4
#define ENB 6
#define IN3 7
#define IN4 8

/* ================= User knobs ================= */
const unsigned TURN_TIMEOUT_MS = 3500;

// PD gains & limits
const float KP_ANG   = 3.6f;   // proportional to angle error (↑ faster, ↑ overshoot risk)
const float KD_VEL   = 0.90f;  // damping with angular velocity (↑ smoother, ↓ overshoot)
const int   PWM_MIN  = 70;     // must overcome motor stiction
const int   PWM_MAX  = 120;    // cap pivot authority
const float DONE_ERR = 1.0f;   // finish if |error| <= 1.0 deg
const float DONE_VEL = 5.0f;   //           and |vel|   <= 5 dps
const unsigned LOOP_MS = 12;   // ~80 Hz control loop

// Short brake at the end (per direction, tweak per rig asymmetry)
const unsigned BRAKE_L_MS = 35;
const unsigned BRAKE_R_MS = 35;

// Asymmetry compensation for biased pivot (real rigs are rarely symmetric)
const int BOOST_PIVOT_R_LFWD = 16; // right pivot: add forward on left motor
const int BOOST_PIVOT_L_RFWD = 14; // left  pivot: add forward on right motor

/* ================= MPU6050 ================= */
const uint8_t MPU_ADDR     = 0x68;
const float   GYRO_SCALE   = 131.0f; // LSB/deg/s @ ±250 dps
const float   VEL_DEADBAND = 0.35f;  // ignore tiny rate to reduce drift
const float   LPF_ALPHA    = 0.22f;  // low-pass filter for vel (0..1)

/**
 * @brief Minimal IMU runtime state.
 * gyroZ_bias : bias (dps) determined at startup
 * velZ_dps_f : filtered angular velocity (dps)
 * angleZ_deg : integrated heading (deg)
 * lastUs     : last timestamp for integration (us)
 */
struct IMUState {
  float gyroZ_bias = -0.218921f; // will be calibrated at boot
  float velZ_dps_f = 0.0f;
  float angleZ_deg = 0.0f;
  unsigned long lastUs = 0;
} imu;

/**
 * @brief One turn "job" for the demo plan.
 * name   : label for printing
 * target : angle target in degrees (+ left / − right)
 */
struct TurnJob {
  const char* name;
  float target;
};

/**
 * @brief Result container of a turn.
 * finalAngle : integrated angle after settle
 */
struct TurnResult {
  float finalAngle;
};

/* ================= Low-level MPU helpers (functions) ================= */

/**
 * @brief Write one byte to MPU6050 register over I2C.
 */
void mpuWrite(uint8_t reg, uint8_t val){
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission(true);
}

/**
 * @brief Read N bytes from an MPU6050 register over I2C into buf.
 */
void mpuRead(uint8_t reg, uint8_t n, uint8_t *buf){
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, n, (uint8_t)true);
  for(uint8_t i=0;i<n && Wire.available();++i) buf[i] = Wire.read();
}

/**
 * @brief Init MPU6050 for gyro-only Z usage.
 * - Wake up, set DLPF ~44 Hz, range ±250 dps.
 */
void mpuInit(){
  mpuWrite(0x6B, 0x00); // wake
  mpuWrite(0x1A, 0x03); // DLPF ~44Hz
  mpuWrite(0x1B, 0x00); // ±250 dps
  delay(50);
}

/**
 * @brief Read raw gyro Z angular rate in dps (unbiased, unfiltered).
 */
float readGyroZ_dps_raw(){
  uint8_t b[2];
  mpuRead(0x47, 2, b);
  int16_t z = (int16_t)((b[0]<<8) | b[1]);
  return (float)z / GYRO_SCALE;
}

/**
 * @brief Calibrate gyro Z bias by averaging 'samples' readings.
 *        Keep the robot still and level during this.
 */
void calibrateGyroZ(IMUState* s, unsigned samples=800){
  double sum=0;
  delay(20);
  for(unsigned i=0;i<samples;i++){
    sum += readGyroZ_dps_raw();
    delay(1);
  }
  s->gyroZ_bias = (float)(sum / (double)samples);
}

/**
 * @brief Reset integrated angle and timestamp (start a fresh integration).
 */
void resetAngle(IMUState* s){
  s->angleZ_deg = 0.0f;
  s->lastUs     = micros();
}

/**
 * @brief Update filtered velocity and integrate heading.
 *        - subtract bias
 *        - deadband very small velocities
 *        - low-pass filter the rate
 *        - integrate to angle using dt in seconds
 */
void updateAngle(IMUState* s){
  unsigned long now = micros();
  float dt = (now - s->lastUs) * 1e-6f;
  s->lastUs = now;

  float vel = readGyroZ_dps_raw() - s->gyroZ_bias;
  if (fabs(vel) < VEL_DEADBAND) vel = 0.0f;

  s->velZ_dps_f += LPF_ALPHA * (vel - s->velZ_dps_f);
  s->angleZ_deg += s->velZ_dps_f * dt;
}

/**
 * @brief Read current integrated angle (deg).
 */
float getAngle(const IMUState* s){ return s->angleZ_deg; }

/* ================= Motor helpers (functions) ================= */

/**
 * @brief Control left motor with signed PWM (positive forward).
 */
void motorLeft(int s){
  if (s > 0){ digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); analogWrite(ENA, s); }
  else if (s < 0){ digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); analogWrite(ENA, -s); }
  else { analogWrite(ENA,0); digitalWrite(IN1,LOW); digitalWrite(IN2,LOW); }
}

/**
 * @brief Control right motor with signed PWM (positive forward).
 */
void motorRight(int s){
  if (s > 0){ digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);  analogWrite(ENB, s); }
  else if (s < 0){ digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); analogWrite(ENB, -s); }
  else { analogWrite(ENB,0); digitalWrite(IN3,LOW); digitalWrite(IN4,LOW); }
}

/**
 * @brief Spin in place with a slight hardware bias compensation.
 * dir : +1 = pivot left,  -1 = pivot right
 * pwm : magnitude of PWM
 */
void spinPivotBiased(int dir, int pwm){
  int pL = pwm, pR = pwm;
  if (dir>0)  pR += BOOST_PIVOT_L_RFWD; // pivot left → right motor forward bias
  else        pL += BOOST_PIVOT_R_LFWD; // pivot right → left  motor forward bias
  if (dir>0){ motorLeft(-pL); motorRight(+pR); }
  else      { motorLeft(+pL); motorRight(-pR); }
}

/**
 * @brief Coast stop both motors.
 */
void motorStop(){ motorLeft(0); motorRight(0); }

/**
 * @brief Short active brake on both motors, then stop.
 * ms : brake duration (per direction can be tuned).
 */
void motorBrakeShort(unsigned ms=30){
  analogWrite(ENA,255); digitalWrite(IN1,HIGH); digitalWrite(IN2,HIGH);
  analogWrite(ENB,255); digitalWrite(IN3,HIGH); digitalWrite(IN4,HIGH);
  delay(ms);
  motorStop();
}

/**
 * @brief Keep integrating for 'ms' to allow angle to settle after braking.
 */
void trackSettle(IMUState* s, unsigned ms=140){
  unsigned long t0=millis();
  while (millis()-t0 < ms){
    updateAngle(s);
    delay(2);
  }
}

/* ================= PD turn core (function) ================= */
/**
 * @brief PD-turn to 'targetDeg'. Returns final integrated angle (deg).
 * - Reset angle integration
 * - Closed-loop PD with rate feedback and near-target PWM reduction
 * - Short brake when both angle error & rate are small
 * - Final settle time for a crisp stop
 */
float turnToAnglePD(IMUState* s, float targetDeg){
  resetAngle(s);
  unsigned long t0 = millis();
  int pwmPrev = 0;

  while (millis() - t0 < TURN_TIMEOUT_MS){
    updateAngle(s);
    float ang = getAngle(s);
    float err = targetDeg - ang;  // deg
    float vel = s->velZ_dps_f;    // dps

    // Finish condition: small error AND small velocity
    if (fabs(err) <= DONE_ERR && fabs(vel) <= DONE_VEL){
      motorBrakeShort(targetDeg>0 ? BRAKE_L_MS : BRAKE_R_MS);
      break;
    }

    // PD control law: positive u → pivot left; negative u → pivot right
    float u = KP_ANG*err - KD_VEL*vel;
    int sgn  = (u >= 0.0f) ? +1 : -1;
    int pwm  = (int)fabs(u);

    // Reduce max PWM as we get close to target
    int maxNear = map((int)constrain((int)fabs(err),0,20), 0, 20, PWM_MIN+2, PWM_MAX);
    if (pwm < PWM_MIN) pwm = PWM_MIN;
    if (pwm > maxNear) pwm = maxNear;

    // Soft ramp-up to avoid jerk
    const int PWM_RAMP_UP = 6;
    if (pwm > pwmPrev + PWM_RAMP_UP) pwm = pwmPrev + PWM_RAMP_UP;
    pwmPrev = pwm;

    spinPivotBiased(sgn, pwm);
    delay(LOOP_MS);
  }

  motorStop();
  trackSettle(s, 140);
  return getAngle(s);
}

/* ================= Demo plan (array + pointer) ================= */
/**
 * @brief Execute a list of turn jobs and print results.
 */
void runTurnPlan(const TurnJob* jobs, size_t n){
  for(size_t i=0;i<n;i++){
    const TurnJob* job = &jobs[i];     // pointer evidence
    delay( (i==0) ? 3000 : 1500 );

    float angle = turnToAnglePD(&imu, job->target);

    TurnResult res{ angle };           // structure usage
    Serial.print(F("# Final ")); Serial.print(job->name);
    Serial.print(F(": ")); Serial.println(res.finalAngle, 2);
  }
}

/* ================= Arduino setup / loop ================= */
void setup(){
  Serial.begin(115200);
  Wire.begin();

  pinMode(ENA,OUTPUT); pinMode(IN1,OUTPUT); pinMode(IN2,OUTPUT);
  pinMode(ENB,OUTPUT); pinMode(IN3,OUTPUT); pinMode(IN4,OUTPUT);
  motorStop();

  mpuInit();
  Serial.println(F("# Calibrating gyroZ... keep still"));
  calibrateGyroZ(&imu, 800);
  resetAngle(&imu);
  Serial.print(F("# gyroZ_bias = ")); Serial.println(imu.gyroZ_bias,5);
  Serial.println(F("# PD turn ready"));
}

void loop(){
  // Demo: two 90° turns (right, then left). Adjust / extend as needed.
  static const TurnJob jobs[] = {
    { "R (-90 deg)", -90.0f },
    { "L (+90 deg)", +90.0f }
  };
  runTurnPlan(jobs, sizeof(jobs)/sizeof(jobs[0]));
  delay(2000);
}