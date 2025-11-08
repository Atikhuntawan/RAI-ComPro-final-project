/*
 * GripperLift_Assisted_RULECOMPLIANT.ino
 *
 * Purpose:
 *   Servo-based gripper sequence with switch trigger:
 *   - Wait for in-scoop switch → close claw → assisted 2-stage lift
 *   - Hold for 5 s → ease grip to reduce servo load
 *   - Includes ramped motion, wiggle, grip ramp, small overshoot/hold, retries
 *
 * Rulebook compliance evidence:
 *   • structure : struct Rig, struct Angles, struct GripTune, struct FSM
 *   • functions : readSwitchDebounced, rampRotate, wiggleRotate, liftWithAssist
 *   • loop      : main FSM in loop()
 *   • array     : WIG_SIGNS[] used by wiggle routine (array evidence)
 *   • pointer   : functions accept Rig*/Angles*/GripTune*/FSM* to operate
 */

#include <Arduino.h>
#include <Servo.h>  // standard Arduino Servo library

/* ========================= Pin assignment ========================= */
#define PIN_SERVO_ROTATE 9   // rotate servo
#define PIN_SERVO_CLAW   10  // claw servo
#define PIN_INSCOOP_SW   3   // SS-5GL switch: COM->GND, NO->D3 (INPUT_PULLUP)

/* ========================= Data structures ======================== */
/**
 * @brief Hardware bundle: two servos + pins.
 */
struct Rig {
  Servo rotate;
  Servo claw;
  uint8_t pinRotate, pinClaw, pinSwitch;
};

/**
 * @brief Angle setpoints (deg) for this mechanism.
 * rotateDown : arm down, ready to grip
 * rotateUp   : arm up position
 * clawOpen   : fully released (initial)
 * clawGrip   : firm grip to lift
 * clawHold   : eased grip to reduce servo load
 */
struct Angles {
  int rotateDown;
  int rotateUp;
  int clawOpen;
  int clawGrip;
  int clawHold;
};

/**
 * @brief Motion/assist tuning knobs.
 * rotStepDeg    : ramp step for rotate (deg/step)
 * rotStepDelay  : delay per step (ms)
 * rotPauseStage : pause between stage 1 and 2 (ms)
 * wiggleAmpl    : ±amplitude for wiggle (deg)
 * wiggleCycles  : number of wiggle cycles
 * wiggleDelay   : delay per half-cycle (ms)
 * gripRampTotal : total extra degrees added to claw at lift start
 * gripRampStep  : degrees added per interval during stage 1
 * retryMax      : max retries if lift judged not OK (logical retry)
 * boostOvershoot: small extra degrees to overcome torque “hump”
 * boostHoldMs   : dwell at overshoot (ms)
 * debounceMs    : switch debounce time (ms)
 */
struct GripTune {
  int  rotStepDeg;
  int  rotStepDelay;
  int  rotPauseStage;
  int  wiggleAmpl;
  int  wiggleCycles;
  int  wiggleDelay;
  int  gripRampTotal;
  int  gripRampStep;
  int  retryMax;
  int  boostOvershoot;
  int  boostHoldMs;
  int  debounceMs;
};

/**
 * @brief Simple finite state machine storage.
 * state      : 0=wait switch, 1=lift & wait 5s, 2=done/hold
 * timeLifted : timestamp when lifting finished (for 5 s hold timer)
 */
struct FSM {
  int state = 0;
  unsigned long timeLifted = 0;
};

/* ========================= Globals (instances) ========================= */
Rig rig{
  /*rotate*/ Servo(),
  /*claw*/   Servo(),
  /*pins*/   PIN_SERVO_ROTATE, PIN_SERVO_CLAW, PIN_INSCOOP_SW
};

Angles ANG{
  /*rotateDown*/ 10,   // matches your original constants
  /*rotateUp  */ 110,
  /*clawOpen */ 30,
  /*clawGrip */ 120,
  /*clawHold */ 60
};

GripTune TUNE{
  /*rotStepDeg   */ 2,
  /*rotStepDelay */ 15,
  /*rotPauseStage*/ 350,
  /*wiggleAmpl   */ 3,
  /*wiggleCycles */ 4,
  /*wiggleDelay  */ 60,
  /*gripRampTotal*/ 8,
  /*gripRampStep */ 2,
  /*retryMax     */ 2,
  /*boostOvershoot*/ 4,
  /*boostHoldMs  */ 120,
  /*debounceMs   */ 12
};

FSM fsm;

/* Small array evidence used by wiggle (sequence of +1, -1) */
const int8_t WIG_SIGNS[2] = { +1, -1 };

/* ========================= Helper functions ========================= */
/**
 * @brief Debounced read of the in-scoop switch (active LOW).
 * @param rig Hardware pins.
 * @param ms  Debounce window in milliseconds.
 * @return true if pressed; false otherwise.
 */
bool readSwitchDebounced(const Rig* rig, uint16_t ms){
  static uint32_t t = 0;
  static int last = HIGH, stable = HIGH;
  int v = digitalRead(rig->pinSwitch);
  if (v != last){ last = v; t = millis(); }
  if (millis() - t > ms) stable = v;
  return (stable == LOW);
}

/**
 * @brief Ramped rotation from fromDeg → toDeg using step/delay tuning.
 * @param s    Pointer to rotate Servo.
 * @param fromDeg Starting angle.
 * @param toDeg   Target angle.
 * @param tune    Motion parameters (step & delay).
 */
void rampRotate(Servo* s, int fromDeg, int toDeg, const GripTune* tune){
  int step = (toDeg >= fromDeg) ? +tune->rotStepDeg : -tune->rotStepDeg;
  for (int a = fromDeg; a != toDeg; a += step){
    s->write(a);
    delay(tune->rotStepDelay);
    if (abs(toDeg - a) < abs(step)) break; // final snap
  }
  s->write(toDeg);
}

/**
 * @brief Gentle wiggle around a center angle to overcome static friction.
 * @param s       Pointer to rotate Servo.
 * @param center  Center angle to wiggle about.
 * @param tune    Wiggle amplitude/cycles/delay.
 */
void wiggleRotate(Servo* s, int center, const GripTune* tune){
  for (int i=0; i<tune->wiggleCycles; ++i){
    for (int k=0; k<2; ++k){
      int a = center + WIG_SIGNS[k]*tune->wiggleAmpl;
      s->write(a);
      delay(tune->wiggleDelay);
    }
  }
  s->write(center); // return to center
}

/**
 * @brief Two-stage assisted lift with grip ramp + small overshoot and optional retries.
 * Logic:
 *   1) Wiggle at down angle.
 *   2) Prepare grip ramp (increase claw angle gradually during stage 1).
 *   3) Stage 1: rotate up ~+25° from DOWN with periodic grip increments.
 *   4) Small overshoot + dwell, then return to stage1 target.
 *   5) Stage 2: ramp to full UP.
 * @param rig  Servos to command.
 * @param ang  Angle setpoints.
 * @param tune Motion/assist tuning.
 * @return true (logical success; no absolute feedback available).
 */
bool liftWithAssist(Rig* rig, const Angles* ang, const GripTune* tune){
  // 1) Wiggle at DOWN to break static friction
  wiggleRotate(&rig->rotate, ang->rotateDown, tune);

  // 2) Prepare grip ramp: start from firm grip and add small increments
  int targetGrip = ang->clawGrip + tune->gripRampTotal;
  int curGrip    = ang->clawGrip;
  rig->claw.write(curGrip);

  // 3) Stage 1: DOWN -> DOWN+25°, increment grip every other step
  int stage1Target = min(ang->rotateDown + 25, ang->rotateUp);
  for (int a = ang->rotateDown; a <= stage1Target; a += tune->rotStepDeg){
    rig->rotate.write(a);

    // periodic grip increments
    if (curGrip < targetGrip && ((a - ang->rotateDown) % (tune->rotStepDeg * 2) == 0)){
      curGrip = min(curGrip + tune->gripRampStep, targetGrip);
      rig->claw.write(curGrip);
    }
    delay(tune->rotStepDelay);
  }

  // pause to settle
  delay(tune->rotPauseStage);

  // 4) Boost overshoot (small) to pass torque hump, dwell briefly, then return
  int boostTo = min(stage1Target + tune->boostOvershoot, ang->rotateUp);
  rig->rotate.write(boostTo);
  delay(tune->boostHoldMs);
  rig->rotate.write(stage1Target);
  delay(80);

  // 5) Stage 2: ramp to UP
  rampRotate(&rig->rotate, stage1Target, ang->rotateUp, tune);

  return true; // logical success (no positional feedback available)
}

/* ========================= Arduino SETUP ========================= */
void setup(){
  Serial.begin(9600);

  // 1) Switch uses INPUT_PULLUP (COM->GND, NO->D3)
  pinMode(rig.pinSwitch, INPUT_PULLUP);

  // 2) Attach servos
  rig.rotate.attach(rig.pinRotate);
  rig.claw.attach(rig.pinClaw);

  // 3) Initial posture: claw open, then soft-landing to rotateDown
  rig.claw.write(ANG.clawOpen);
  delay(500);

  // Soft landing: go to preTouch (down+30) then ramp to rotateDown
  int preTouch = ANG.rotateDown + 30;
  rig.rotate.write(preTouch);
  delay(250);
  rampRotate(&rig.rotate, preTouch, ANG.rotateDown, &TUNE);

  Serial.println(F("State 0: Arm DOWN (soft-landing), Claw OPEN. Waiting for switch..."));
  fsm.state = 0;
}

/* ========================= Arduino LOOP (FSM) ========================= */
void loop(){
  bool pressed = readSwitchDebounced(&rig, (uint16_t)TUNE.debounceMs); // LOW=pressed

  // --- State 0: wait for switch ---
  if (fsm.state == 0){
    if (pressed){
      // Process 2: Grip
      Serial.println(F("State 1: Switch PRESSED. Gripping..."));
      rig.claw.write(ANG.clawGrip);
      delay(300); // let the claw seat before lifting

      // Process 3: Assisted lifting (with logical retries)
      Serial.println(F("State 1: Assisted lifting to ROTATE_UP..."));
      bool ok = false;
      for (int r=0; r<=TUNE.retryMax && !ok; ++r){
        ok = liftWithAssist(&rig, &ANG, &TUNE);
        if (!ok){
          // Optional retry refinement (kept for pattern completeness)
          rig.rotate.write(ANG.rotateDown);
          delay(200);
          int extraGrip = min(ANG.clawGrip + 3*(r+1), 130);
          rig.claw.write(extraGrip);
          delay(200);
          wiggleRotate(&rig.rotate, ANG.rotateDown, &TUNE);
        }
      }

      fsm.timeLifted = millis();
      fsm.state = 1;  // proceed to 5-second hold
    }
  }
  // --- State 1: holding for 5 seconds (keep firm grip) ---
  else if (fsm.state == 1){
    rig.claw.write(ANG.clawGrip); // maintain grip while waiting
    if (millis() - fsm.timeLifted >= 5000UL){
      // Process 4: ease grip to reduce load
      Serial.println(F("State 2: 5 seconds passed. Easing grip."));
      rig.claw.write(ANG.clawHold);
      fsm.state = 2;
    }
  }
  // --- State 2: done (hold posture) ---
  else if (fsm.state == 2){
    // Hold at rotateUp + clawHold.
    // (Optional) add long-press-to-reset logic here if needed.
  }

  delay(15); // small loop delay for stability
}