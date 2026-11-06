// ======================================================
// Config.h  -  MISSION 2  "TEACH & REPLAY"
// Hardware + START values. Everything you TEACH is saved in EEPROM by the
// serial commands - you do NOT need to edit this file again.
// ======================================================
#ifndef M2_CONFIG_H
#define M2_CONFIG_H

#define BUILD_NAME "MISSION 2 - TEACH & REPLAY v18 (anti-slip + slot position fix)"
#define SAFE_MODE 1

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_PWMServoDriver.h>

// ---------------- DRIVE MOTORS (tested) ----------------
#define M1_PWM 2
#define M1_IN1 31
#define M1_IN2 30
#define M2_PWM 3
#define M2_IN1 32
#define M2_IN2 33
#define M3_PWM 4
#define M3_IN1 35
#define M3_IN2 34
#define M4_PWM 5
#define M4_IN1 36
#define M4_IN2 37
#define M4_REVERSED 1

// v17 ANTI-SLIP: lower top speed, soft start, longer slow end
#define SPEED 145                // og:125
#define SLOW_SPEED 85           // og: 85
#define RAMP_START_PWM  80       // og:60
#define RAMP_MS        350       // og:500
#define COUNTS_PER_MM 20.3718

// start values - change with the serial commands  ffwd / fstrafe
#define DEF_FORWARD_FACTOR  1.100
#define DEF_STRAFE_FACTOR   1.414
#define DIAGONAL_FACTOR     1.750

#define ROTATION_MIRRORED 1
#define TURN_90_WHEEL_MM 380
#define ENCODER_TOLERANT  1     // v17: distance = middle 2 of the 4 wheels -> a SLIPPING wheel is ignored

// ---------------- MPU6050 GYRO (tested) ----------------
#define USE_MPU           1
#define MPU_ADDR          0x68
#define MPU_REQUIRED      1
#define GYRO_AXIS         2
#define GYRO_SIGN        -1
#define MPU_CAL_SAMPLES   400
#define HEADING_KP        0.040
#define HEADING_KD        0.004
#define HEADING_MAX       0.25
#define ASSIST_KP_PWM     14.0
#define ASSIST_KD_PWM     2.5
#define ASSIST_KI_PWM     12.0
#define ASSIST_I_MAX      35
#define ASSIST_MAX_PWM    60
#define ASSIST_SIGN        1
#define MPU_GLITCH_DPS    300.0
#define HEADING_FIX_ENABLE   1
#define HEADING_FIX_TOL_DEG  0.8
#define STRAFE_RIGHT_FWD_TRIM_PWM   0
#define STRAFE_LEFT_FWD_TRIM_PWM    0
#define SYNC_KP_GYRO      0.15
#define TURN_TOL_DEG      0.4
#define TURN_FINE_PWM     110
#define TURN_FINE_MS_PER_DEG 30

// ---------------- SPEED CONTROL (only if SAFE_MODE 0) ----------------
#define SPEED_CONTROL     (SAFE_MODE ? 0 : 1)
#define REF_CPS_DEFAULT   5000
#define V_MAX_FRAC        0.85
#define V_SLOW_FRAC       0.30
#define V_CREEP_FRAC      0.12
#define A_ACC_FRAC        2.5
#define A_DEC_FRAC        2.0
#define CTRL_MS           20
#define SPEED_FILTER      0.4
#define SPEED_GAIN        0.25
#define SYNC_KP           3.0
#define START_PWM         90
#define BRAKE_MS         (SAFE_MODE ? 0 : 60)
#define TRIM_ENABLE       (SAFE_MODE ? 0 : 1)
#define TRIM_TOL_MM     2.0
#define TRIM_MAX_MS    1500

// ---------------- ARM (PCA9685 0x40) ----------------
#define PCA9685_ADDR  0x40
#define BASE_SERVO    0
#define BOTTOM_SERVO  1
#define TOP_SERVO     2
#define BASE_MIN    100
#define BASE_MAX    560
#define BOTTOM_MIN  150
#define BOTTOM_MAX  510
#define TOP_MIN     270
#define TOP_MAX     530

// START poses (teach with:  pose idle / pose init / pose scan / pose scanl)
#define IDLE_BASE      331
#define IDLE_BOTTOM    330
#define IDLE_TOP       530
#define INITIAL_BASE   320
#define INITIAL_BOTTOM 365
#define INITIAL_TOP    480
#define SCAN_BASE      195       // arm to the RIGHT, camera looks down
#define SCAN_BOTTOM    300
#define SCAN_TOP       320
#define SCAN_LEFT_BASE 445       // arm to the LEFT
#define PICK_BOTTOM    441       // first guess of the pick pose (each slot is taught)
#define PICK_TOP       270
#define PLACE_BASE     335       // first guess of the place pose (each zone is taught)
#define PLACE_BOTTOM   470
#define PLACE_TOP      270

#define HOVER_FRACTION   0.30    // arm first goes to a point 30% back towards the scan pose
#define ARM_MOVE_MS      1200
#define ARM_DOWN_MS      1500
#define PICK_DWELL_MS    2500    // nozzle stays ON the patient (suction builds up)
#define PICK_LIFT_MS     1500    // slow lift
#define PLACE_DWELL_MS    800

// ---------------- SUCTION (pump switch board, servo pulse) ----------------
#define SUCTION_CHANNEL       7       // <<< YOUR PUMP CHANNEL
#define SUCTION_PULSE_ON_US   2000
#define SUCTION_PULSE_OFF_US  1000

// ---------------- GRIPPER: CLAW (v10) ----------------
// 1 = servo CLAW on PCA9685 channel CLAW_CHANNEL (the pump on channel 7 is kept OFF)
// 0 = old suction pump
#define GRIPPER_CLAW      1
#define CLAW_CHANNEL     12
// START angles. Change on the robot - no upload needed:
//    clawset 45 180   (OPEN  CLOSE)  -> saved in EEPROM
//    claw open / claw close / claw 120   (move it now, to test / find the angle)
#define CLAW_OPEN_DEG    45
#define CLAW_CLOSE_DEG  180
#define CLAW_SERVO_MIN  110      // PCA9685 pulse for 0 deg   (same as the kit servos)
#define CLAW_SERVO_MAX  510      // PCA9685 pulse for 180 deg
#define CLAW_MOVE_MS    600      // time the claw needs to open / close
#define CLAW_HOLD_MS    400      // extra wait after closing, before the arm lifts
// v13: CLAW APPROACH - the claw lands SHORT of the patient, slides onto it, THEN closes
//   1. robot moves back CLAW_BACKOFF_MM (arm up)  -> claw lands short of the patient
//   2. arm down, claw open
//   3. robot moves CLAW_PUSH_MM "in front" (arm stays down) -> open claw slides around the patient
//   4. claw closes, arm lifts, robot moves back to the aimed point for the camera check
#define CLAW_APPROACH        1   // 1 = on   0 = close the claw right where it lands (v12)
#define CLAW_BACKOFF_MM     25   // land this much SHORT of where it lands now (mm)
#define CLAW_PUSH_MM        50   // then slide the open claw this much "in front" (mm)
#define CLAW_APPROACH_AXIS   1   // "in front" = 1: along the arm (wheels LEFT/RIGHT - the arm points sideways)
                                 //              2: robot FORWARD/BACK
#define CLAW_APPROACH_SIGN   1   // 1 = away from the robot along the arm (axis 1) / forward (axis 2)
// v18f: ONE special slot gets its own claw approach + aims AHEAD after some failed tries
#define SPECIAL_SLOTS {3, 4, 5, 6} // slot numbers 1-12 that get the special approach
#define SPECIAL_BACKOFF_MM   40   // land this much short (normal CLAW_BACKOFF_MM)
#define SPECIAL_PUSH_MM      80   // then slide this much in front (normal CLAW_PUSH_MM)
#define SPECIAL_AHEAD_AFTER   4   // after this many failed tries ...
#define SPECIAL_AHEAD_MM     45   // ... aim this much AHEAD of the taught spot (mm)
#define SPECIAL_AHEAD_AXIS    1   // 1 = "ahead" along the arm (same as claw approach)  2 = robot FORWARD
                                 // -1 = the other way
#if GRIPPER_CLAW
#define RELEASE_WIGGLE       0   // claw: no shaking needed at the drop
#define RELEASE_WAIT_MS    800   // claw: arm stays down this long after OPEN
#endif

// ---------------- KIT SERVOS (SG90, kick) ----------------
// PCA9685 channel of each kit servo
#define KIT_CH_HOSPITAL   14
#define KIT_CH_PCC1       13
#define KIT_CH_PCC2       15
// START angles (degrees). Change them on the robot - no upload needed:
//    kitset h 90 60      (zone  REST  KICK)   -> saved in EEPROM
//    kitdeg h 75         (move that servo to 75 now, to find the angle)
//    kitreset            (back to the values below)
#define KIT_HOSP_REST     90
#define KIT_HOSP_KICK     60
#define KIT_PCC1_REST     90
#define KIT_PCC1_KICK     65
#define KIT_PCC2_REST     152
#define KIT_PCC2_KICK    128
#define KIT_KICK_MS      600
#define KIT_RETURN_MS    400
#define KIT_KICK_TIMES     1
#define KIT_SERVO_MIN    110
#define KIT_SERVO_MAX    510

// ---------------- CAMERA (Grove Vision AI V2, 240 x 240) ----------------
#define CLASS_RED        0
#define CLASS_YELLOW     1
#define CLASS_GREEN      2       // class 3 = "sample" (ignored)
#define MIN_SCORE        50
#define CONFIRM_FRAMES   3
#define SCAN_TIMEOUT_MS  3000
#define IMAGE_CENTER_X   120
#define IMAGE_CENTER_Y   120
#define LOOK_MS          900
#define SETTLE_MS        350

// ---- box centre from x / y / w / h (v6) ----
#define IMAGE_W          240
#define IMAGE_H          240
#define BOX_XY_IS_CENTER   1     // 1 = camera sends the box CENTRE as x,y (Grove Vision AI V2 / SSCMA)
                                 // 0 = camera sends the TOP-LEFT corner -> centre = x + w/2, y + h/2
                                 //     (check with "live": see the README, after changing -> teach again)
#define EDGE_MARGIN_PX     2     // box touching the picture edge = patient cut off -> centre wrong -> not used for aiming
#define ALIGN_TO_IMAGE_CENTER 0  // 0 = aim at the pixel recorded in "teach" (recommended)
                                 // 1 = aim at the picture middle (only if the camera looks straight down the nozzle)
#define ALIGN_FRAMES       5     // precise look: middle value (median) of this many pictures
#define ALIGN_MIN_FRAMES   3
#define ALIGN_LOOK_MS   2000

// fine correction before each pick: arm base (one direction) + wheels (the other direction)
#define FINE_ENABLE        1
#define FINE_TOL_PX        3
#define FINE_ROUNDS        5
#define FINE_MAX_UNITS    15
#define FINE_GAIN        0.9
#define CAL_BASE_UNITS    10
#define FINE_WHEELS        1     // 1 = also strafe the robot a few mm (needs "teach N" again once)
#define CAL_STRAFE_MM     10     // teach: strafe this much to learn how the picture moves
#define FINE_WHEEL_MIN_MM 1.5    // smaller wheel corrections are skipped
#if GRIPPER_CLAW
#define FINE_MAX_MM       25     // never correct more than this with the wheels (per direction)
                                 // (claw: a missed push can move the patient ~20 mm)
#else
#define FINE_MAX_MM       12     // never correct more than this with the wheels (per direction)
#endif
// v11: fine correction picks the 2 directions that move the picture most differently:
//      arm base, wheels LEFT/RIGHT, wheels FORWARD/BACK (all learned in "teach N")
#define FINE_MIN_SIN      0.5    // two directions this different (sin of angle) = use them (prefer the arm base)
#define FINE_MIN_PROGRESS_PX 1.5 // error did not get this much smaller after a move -> stop correcting

// pick check
#define PICK_MAX_TRIES     8     // search pattern around the aimed point (max 9 = full 3 x 3)
#if GRIPPER_CLAW
#define PICK_STEP_MM       6     // distance between the search points (mm) - claw grips a wider area
#else
#define PICK_STEP_MM       3     // distance between the search points (mm)
#endif
#define PICK_STEP_BASE     2     // arm-base units per step if the slot has no wheel-cal
#define PICK_SHIFT_WHEELS  1     // 1 = search points also left/right with the wheels, 0 = arm base only
#define PICK_RETRY_PRESS   2     // retries press the nozzle this much lower (bottom servo units)
// v18d: GREEN patients - arm goes this much LOWER when picking (same units as PICK_RETRY_PRESS)
#define GREEN_PRESS_BOTTOM 3     // + = lower (bottom servo units). 0 = same as other colours
#define GREEN_PRESS_TOP    0     // top servo extra (leave 0 unless the claw angle looks wrong)
// v18d: TIGHT slots - camera aims more exactly before the pick (slot numbers 1-12)
#define TIGHT_SLOTS        {3, 4, 5, 6}   // e.g. {3, 9} for more slots
#define TIGHT_FINE_TOL_PX  1     // normal FINE_TOL_PX
#define TIGHT_FINE_ROUNDS  8     // normal FINE_ROUNDS
#define TIGHT_POSFIX_TOL_MM 0.8  // normal POSFIX_TOL_MM
#define TIGHT_POSFIX_ROUNDS 6    // normal POSFIX_ROUNDS
// v18e SAFETY before the arm goes down (stops pushing the neighbour patients)
#define PICK_MAX_HEADING_DEG 3.0 // robot turned more than this (gyro) -> fix heading, still off = no pick
#define PICK_MAX_BASE_DEV   10   // arm base more than this many units away from the TAUGHT base = no pick
// v18i: every pick try
#define PICK_PAUSE_MS    100UL // wait this long before the arm goes down (0 = no wait)
#define PICK_FWD_OFFSET_MM  2.0  // robot moves this much FORWARD before the pick (- = back), undone after
// v8: "PICK OK" only with PROOF: the camera must answer on VERIFY_CLEAR_FRAMES pictures
// and NONE of them may show a patient near the pick spot. One sighting = NOT picked.
#define VERIFY_MS          3000  // max time for the check
#define VERIFY_CLEAR_FRAMES   4  // answered pictures needed to say "gone"
#define VERIFY_MIN_SCORE     30  // lower than MIN_SCORE: a tilted / pushed patient still counts
#define VERIFY_RADIUS_PX     80  // search circle around the pick spot (at least this) - big: a missed claw PUSHES the patient away
#define VERIFY_RADIUS_SIZE  2.2  // ... or 2.2 x the patient size (w / h)
#define VERIFY_SETTLE_MS    500  // arm still before the check (sharp picture)
#define VERIFY_FRAMES         2  // (old check, not used any more)
// v12: CLAW_HELD_VISIBLE 0 = the camera is mounted so it can NOT see a patient held in the claw.
//      -> after the lift, ANY patient at the pick spot is ON THE FLOOR = NOT picked (no swing test).
//      1 = camera can see the held patient -> big-box filter + "floor or claw?" swing test (v11)
#define CLAW_HELD_VISIBLE     1   // v14: ON - your camera DOES see the patient held in the claw
                                  // (with 0 it saw the held patient, thought "still on the floor" and dropped it)
#define VERIFY_HELD_SIZE    1.5  // claw: a patient box this many times bigger than on the floor = held in the claw
#define HELD_TEST_UNITS      40  // claw: arm-base swing for the "floor or claw?" test (units)
#define HELD_SAME_PX         25  // claw: patient still within this many px after the swing = held in the claw
// v11 "floor or claw?" swing: the floor patient should slide HELD_SHIFT_PX in the picture.
// Seen at the SAME pixel = held.  Seen where a FLOOR patient must be = on the floor.  Neither = not sure.
#define HELD_SHIFT_PX        60  // swing size in picture pixels (needs the camera base cal)
#define HELD_FLOOR_PX        30  // patient within this many px of the predicted floor place = on the floor
#define HELD_MIN_HITS         3  // pictures needed for "held" or "on the floor" (of 6 fresh pictures)
#define VERIFY_SKIP_FRAMES    2  // v14: pictures thrown away after the arm moved (the camera can send an OLD picture)

// ---------------- SLOT POSITION FIX (v18) - removes wheel-slip drift at EVERY pick ----------------
// At each slot the camera compares where the patient is with where it was in "teach N"
// (same arm pose). The WHEELS move the robot until it matches, then the map (navX / navY)
// is SET to the slot position. So slip from all earlier trips is thrown away here.
// Needs the wheel calibration from "teach N" (show: "cal: L/R OK  F/B OK").
#define POSFIX_ENABLE      1
#define POSFIX_TOL_MM    1.5     // close enough (mm, each direction)
#define POSFIX_ROUNDS      4     // max correction moves
#define POSFIX_GAIN      0.9
#define POSFIX_MAX_MM   60.0     // bigger first error = wrong patient / not sure -> no fix
#define POSFIX_MIN_MOVE  1.0     // smaller wheel moves are skipped

// ---------------- CAPTURE: camera POSITION FIX at a slot (v15) ----------------
// "capture N": arm pose that sees patient N in the picture (taught once, robot does NOT move).
// In the run, at slot N the arm goes to that pose, the camera measures where the patient is,
// the WHEELS move the robot until the patient is at the recorded pixel again, and then the
// map (navX / navY) is SET to the slot position -> wheel slip / encoder / gyro drift is removed.
#define CAPTURE_ENABLE     0     // 0 = never use the capture fix (OFF for now)
#define CAP_TOL_MM       1.5     // close enough (mm, each direction)
#define CAP_ROUNDS         4     // max correction moves
#define CAP_GAIN         0.9     // move 90% of the measured error each round
#define CAP_MAX_MM      40.0     // bigger error = something is wrong (wrong patient?) -> no correction
#define CAP_SIZE_TOL    0.35     // patient size must be within 35% of the recorded size
#define CAP_CAL_MM      10.0     // capture: robot moves this much LEFT and FORWARD to learn px per mm
#define CAP_MIN_MOVE_MM  1.0     // smaller wheel moves are skipped

// ---------------- MAP  (0,0) = H CENTER, +X = forward (hospital), +Y = left ----------------
#define START_DELAY_MS   7000UL     // wait after START (gyro calibrates in this time)
 // v18i: EXTRA wait after START, added on top (total 15 s)
// START PATH (robot stands in the start zone facing the CENTRE-NODE side, NOT the hospital):
//   strafe RIGHT leg1 -> FORWARD leg2 -> strafe RIGHT leg3 -> TURN RIGHT 90 -> faces hospital on H CENTER
#define DEF_LEG1   236.0      // start: strafe RIGHT out of the start zone   command: leg1
#define DEF_LEG2   432.0      // start: FORWARD up to the H line              command: leg2
#define DEF_LEG3   215.0      // start: strafe RIGHT onto H CENTER            command: leg3
// START PATH CORRECTIONS (mm), added to the saved legs (leg1 / leg2 / leg3 in "show").
// Measured after "node" (robot facing the hospital):
//   2.5 cm too far FORWARD (towards hospital) -> right strafes 25 mm too long -> leg3 -25
//   4 cm too far RIGHT                         -> forward leg 40 mm too short  -> leg2 +40
// Tuning:  X mm too far forward -> LEG3_ADJ - X     X mm too far back  -> LEG3_ADJ + X
//          X mm too far right   -> LEG2_ADJ + X     X mm too far left  -> LEG2_ADJ - X
#define START_LEG1_ADJ   -50.0   // (from the video: 1st strafe shorter, away from the patients)
#define START_LEG2_ADJ   +65.0
#define START_LEG3_ADJ   +73.0   // was +50, now 25 mm less
#define START_TURN_RIGHT 1    // 1 = turn right 90 deg on H CENTER (face hospital), 0 = no turn
#define RUN_KITS         1    // 1 = kits are kicked before the patients, 0 = patients only

#define HOSP_X      255.0
#define HOSP_Y        0.0
#define PCC1_X      255.0
#define PCC1_Y      320.5
#define PCC2_X      255.0
#define PCC2_Y     -301.5
#define RECOV_X    -340.0     // back ~340 mm from H CENTER (yellow path)
#define RECOV_Y     -50.0

// patient dots (CAD). Rows: X = +100 (hospital side) and -100 (start side)
#define ROW_HOSP_X        100.0
#define ROW_START_X      -100.0
#define RIGHT_Y1  -221.5
#define RIGHT_Y2  -321.5
#define RIGHT_Y3  -421.5
#define LEFT_Y1    259.5
#define LEFT_Y2    359.5
#define LEFT_Y3    459.5
// v18c: per-slot BACK adjust (mm). + = robot stops further BACK (towards start zone)
//                  slot: 1  2   3   4  5   6   7 8 9 10 11 12
#define SLOT_BACK_ADJ    {0, 50, 50, 0, 50, 50, 0,0,0, 0, 0, 0}
// v18i: per-slot HOSPITAL adjust (mm). + = robot stops further FORWARD (towards the hospital)
//                  slot: 1  2  3  4  5  6  7  8  9 10 11 12
#define SLOT_HOSP_ADJ    {2, 2, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0}
#define PICK_REACH_MM   241.0    // robot stops this far beside the dot (arm reaches sideways)

// v18b RETURN ASSIST: extra mm on the way BACK from a zone to H CENTER (+ = go further on that leg).
// The way back is: first SIDEWAYS (strafe), then FORWARD/BACK.
//   coming back from PCC2: strafe LEFT, then BACK      coming back from PCC1: strafe RIGHT, then BACK
//   coming back from HOSPITAL: only BACK               coming back from RECOVERY: strafe LEFT, then FORWARD
// Robot stops SHORT of H CENTER on that leg -> make it bigger (e.g. +15). Goes PAST it -> negative.
#define RET_HOSP_SIDE_ADJ    0.0
#define RET_HOSP_FB_ADJ      0.0
#define RET_PCC1_SIDE_ADJ    0.0
#define RET_PCC1_FB_ADJ      0.0
#define RET_PCC2_SIDE_ADJ    0.0     // PCC2 -> H CENTER: strafe LEFT (mm extra)
#define RET_PCC2_FB_ADJ      0.0     // PCC2 -> H CENTER: BACK (mm extra)
#define RET_RECOV_SIDE_ADJ   0.0
#define RET_RECOV_FB_ADJ     -50.0

// v18h DROP BACK-OFF: when DROPPING A PATIENT the robot stops this much SHORT (mm, towards
// H CENTER / the start zone) of the zone point -> the arm does not reach so far forward.
// Only patient drops - the kits and "tz" teaching still go to the full zone point.
#define DROP_BACK_HOSP_MM   100.0
#define DROP_BACK_PCC1_MM   100.0
#define DROP_BACK_PCC2_MM   100.0
#define DROP_BACK_RECOV_MM    0.0

#define NAV_TOL_MM        2.0
#define DROP_SPREAD_MM   45.0

// ---------------- BUTTONS / I2C ----------------
#define START_BUTTON_PIN 22
#define STOP_BUTTON_PIN  29
#define I2C_SDA_PIN      20
#define I2C_SCL_PIN      21

#define SZ(d) ((d) > 150 ? 70 : 15)   // v17: slow end of a move (mm) - was 40 / 10

// ======================================================
// TYPES
// ======================================================
enum MoveType { DIAGONAL_FRONT_RIGHT, ROTATE_RIGHT, STRAFE_RIGHT, BACKWARD, STRAFE_LEFT, FORWARD, ROTATE_LEFT };
enum PatientColor { COL_NONE = 0, COL_RED, COL_GREEN, COL_YELLOW };
enum Zone { ZONE_HOSPITAL = 0, ZONE_PCC1, ZONE_PCC2, ZONE_RECOVERY, ZONE_COUNT };

struct Pose3 { int b, bo, t; };

#define MAX_BOXES 10
struct Box { int x, y, w, h, score, target; };

#define SLOT_COUNT 12
struct SlotData {            // one patient dot
  byte ok;                   // taught
  byte hasPix;               // camera pixel recorded
  float dx, dy;              // taught robot offset from the nominal stop (mm)
  Pose3 pick;                // arm pose with the nozzle ON the patient
  int px, py;                // where the camera sees this patient (scan pose)
};

struct CapData {             // v15: one capture (camera position fix) per slot
  byte ok;
  Pose3 pose;                // arm pose for the picture
  int x, y, w, h;            // where + how big the patient is in the picture (centre, px)
  float lx, ly;              // picture shift per mm robot LEFT   (px / mm)
  float fx, fy;              // picture shift per mm robot FORWARD (px / mm)
};

struct ZoneData {            // one drop zone
  byte ok;
  float dx, dy;              // taught robot offset from the nominal zone point (mm)
  Pose3 place;               // arm pose to drop
};

#define KIT_COUNT   3          // [0] = HOSPITAL  [1] = PCC1  [2] = PCC2   (same order as Zone)
#define KIT_OK_MAGIC 0xA7
#define CLAW_OK_MAGIC 0xB5

#define STORE_MAGIC 0xC5A7
struct Store {
  uint16_t magic;
  Pose3 idle, init, scan;
  int scanLeftBase;
  SlotData slot[SLOT_COUNT];
  ZoneData zone[ZONE_COUNT];
  float bx, by;              // camera: px per arm-base unit
  byte baseCal;
  float fwdF, strafeF;
  float leg1, leg2, leg3;
  byte kitsPCC;
  float backF;               // backward factor (command: fback)
  // v6: wheel fine correction, learned per slot by "teach N". New fields only AFTER this line.
  float sx[SLOT_COUNT], sy[SLOT_COUNT];   // camera: px per mm strafe LEFT
  byte strafeCal[SLOT_COUNT];
  // v9: kit servo angles (command: kitset). New fields only AFTER this line.
  byte kitRest[KIT_COUNT], kitKick[KIT_COUNT];
  byte kitOk;
  // v10: claw angles + pick check on/off (commands: clawset / verify). New fields only AFTER this line.
  byte clawOpen, clawClose;
  byte pickVerify;           // 1 = camera checks every pick (repeat if missed), 0 = trust the claw
  byte clawOk;
  // v11: forward/back wheel calibration per slot (teach N). New fields only AFTER this line.
  float fwx[SLOT_COUNT], fwy[SLOT_COUNT];   // camera: px per mm FORWARD
  byte fwdCal[SLOT_COUNT];
  // v15: capture = camera position fix per slot (command: capture N). New fields only AFTER this line.
  CapData cap[SLOT_COUNT];
  byte capMagic;
};
#define CAP_MAGIC 0x5C

// ======================================================
// SHARED
// ======================================================
extern Store st;
extern Adafruit_PWMServoDriver pwm;
extern bool missionAborted, suctionActive, stepMode, mpuOk, visionOk;
extern bool moveAllowStall, lastMoveHitWall, assistEnabled, assistLog;
extern byte stepCounter;
extern float armBase, armBottom, armTop;
extern Box boxes[MAX_BOXES];
extern int boxCount;
extern float refCps, yawDeg, yawRate, headingRef, moveYaw0;
extern uint8_t mpuAddress;
extern float assistSumAbs, assistMaxAbs;
extern unsigned int assistN, mpuFailTotal, mpuGlitchTotal;
extern PatientColor lastColor;
extern float navX, navY;
extern unsigned long missionT0;

// Mission22.ino
bool abortRequested();
bool stepWait();
bool mgate(const __FlashStringHelper *name);
bool mdrive(MoveType t, float mm, float slow);
bool readLine(char *buf, byte n, bool blocking);
void i2cScan();
void preflight();

// Movement.ino
void encodersBegin();
void readEnc(long r[4]);
void resetEncoders();
void motorRun(byte pwmPin, byte in1, byte in2, int speed);
void stopMotors();
void brakeMotors(unsigned int ms);
void settleWait(unsigned long ms);
float moveProgress(MoveType type);
void wheelDirs(MoveType type, int8_t d[4]);
bool moveFinish(MoveType type, float targetCounts);
bool encoderMoveOpen(float mm, float factor, MoveType type, float slowDistance);
bool encoderMoveClosed(float mm, float factor, MoveType type, float slowDistance);
void speedLoad();
void speedTest();
void wheelTest();
void runMovement(MoveType type, int spd);
void runMovementAssist(MoveType type, int spd);
bool fixHeading();
bool encoderMove(float mm, float factor, MoveType type, float slowDistance);
const char *moveName(MoveType type);
float moveFactor(MoveType type);
bool smallMove(MoveType type, float mm);
bool drive(MoveType type, float mm, float slowDistance);

// Mpu.ino
bool mpuBegin();
bool mpuCalibrate();
void mpuUpdate();
void mpuTest();

// Vision.ino
bool gv2Present();
int gv2Invoke();
bool isPatientClass(int t);
PatientColor classToColor(int t);
const char *colorName(PatientColor c);
void setAim(int x, int y);
bool lookForSample(int &x, int &y, unsigned long timeoutMs);
bool sampleNear(int tx, int ty, int radius, unsigned long timeoutMs, int &ox, int &oy);
int pickCheck(int tx, int ty, int radius, int &ox, int &oy);
extern int lastBoxW, lastBoxH;
bool lookPrecise(int &x, int &y, int &w, int &h, unsigned long timeoutMs);
bool boxCut(const Box &b);
void cameraTest();

// Store.ino
void storeDefaults();
void kitDefaults();
void kitShow();
void capShow();
void clawDefaults();
void clawShow();
void storeLoad();
void storeSave();
void storeShow();
Pose3 slotDefaultPose(byte s);

// Arm.ino
void armApply();
bool armMoveTo(int b, int bo, int t, unsigned long ms);
bool armToPose(const Pose3 &p, unsigned long ms);
bool armToInitial();
bool armToIdle();
bool armScanAt(int base);
bool armLiftHere();
bool armOver(float b, float bo, float t);
bool armDown(float b, float bo, float t);
void setSuction(bool on);
bool waitMs(unsigned long ms);
void kitsHoldAll();
bool kitRelease(Zone z);
void kitMoveDeg(Zone z, int deg);
byte kitChannel(Zone z);
void clawMoveDeg(int deg);

// Nav.ino
bool navRelX(float mm);
bool navRelY(float mm);
bool navGoTo(float x, float y, const __FlashStringHelper *name);
bool navReturnNode();
bool navStartToNode();
bool slotIsRight(byte s);
void slotStop(byte s, float &x, float &y);
bool navToSlot(byte s);
bool navToZone(Zone z, byte dropIndex);
bool navToDrop(Zone z, byte dropIndex);
const __FlashStringHelper *zoneName(Zone z);
Zone zoneFor(PatientColor c, bool rightSide);

// Teach.ino
bool teachSlot(byte s);
bool teachZone(Zone z);
bool teachPose(byte which);
bool captureTeach(byte s);

// Mission.ino
void missionReset();
bool gyroStart(unsigned long waitTotalMs);
bool lookSlot(byte s, PatientColor &c);
bool pickSlot(byte s, PatientColor &c);
bool placeInZone(Zone z, byte k, byte n);
bool processSlot(byte s);
bool deliverKits();
bool runFromNode(bool kits);
bool runMission();
bool startKitsOnly();
void lookAll();
bool zoneTour();
bool slotTour();
void printSummary();
bool captureFix(byte s);
extern bool sRun;
bool sPause(const __FlashStringHelper *done);

#endif