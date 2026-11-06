MISSION 2 - TEACH & REPLAY   (start here)
==========================================
Serial Monitor: 115200, line ending "Newline". Type a command + Enter.
STOP = STOP button (pin 29) or type x

ONLY CHANGE IN Config.h:  SUCTION_CHANNEL (your pump channel). Nothing else.

---------------- DAY 1: SETUP (all saved, never again) ----------------
1. pre                 -> arm driver, camera, gyro OK?
2. suck                -> pump ON, again "suck" -> OFF
3. cam                 -> patient under camera: RED / YELLOW / GREEN correct?
4. mfwd 300 / mleft 300  -> measure. Wrong? ffwd <new>  /  fstrafe <new>
                          new = old x 300 / measured   (show = see old)
5. Robot on H CENTER, patients on all dots.
   teach 1   -> robot drives to slot 1, arm goes down.
                Type letters + Enter to move:
                  wheels  w s a d      arm  j l (base)  i k (bottom)  u o (top)
                  z = small arm steps, v = suction test, c = camera look
                When the 20 mm nozzle sits EXACTLY on the patient: y
                Robot saves it and drives back to H CENTER.
   teach 2 ... teach 12   (same)
6. tz h   tz p1   tz p2   tz r   -> teach the 4 drop zones the same way.
7. show   -> all slots / zones TAUGHT?

---------------- v6: PRECISE AIM (box centre with w + h, wheels correct too) ----------------
A. Check what x,y means:  live  -> slide a patient to the LEFT edge of the picture.
     x stays about half the width (w/2) when the patient touches the edge -> x is the CENTRE
       -> keep BOX_XY_IS_CENTER 1 (normal for Grove Vision AI V2)
     x goes down to about 0 -> x is the TOP-LEFT corner -> set BOX_XY_IS_CENTER 0, upload
   "CUT" in live = box touches the picture edge -> not used for aiming.
B. Teach every slot ONCE more:  teach N  ->  if the nozzle is already right just type y.
   The robot now strafes 10 mm left and back = wheel calibration. show -> "wheel-cal OK".
C. pick N  ->  watch the FINE lines: error goes down to <= 3 px, shown also in mm.
D. SEARCH PATTERN (v7): if the pick fails, the next tries go 3 mm around the aimed point:
        7  2  5        1 = aimed point, then 2..8   (rows = wheels left/right,
        3  1  4                                      columns = arm base)
        8  .  6        Config.h: PICK_MAX_TRIES 8, PICK_STEP_MM 3, PICK_SHIFT_WHEELS 1
   Watch "PICK TRY n/8 base .. wheels .." - the try that works tells you which way the
   taught point is off. Fix that slot with teach N (move the nozzle that way, then y).

E. PICK CHECK (v8): "PICK OK" only with proof - camera answered 4 pictures and NONE shows a
   patient near the spot (score >= 30, circle = 1.2 x patient size, min 40 px).
   Seen once -> NOT picked. Camera silent -> NOT picked. Tune in Config.h: VERIFY_*

---------------- TEST ----------------
pick 1        one patient (robot on H CENTER)
pick 2 ...    one by one
right         patients 1-6        left   7-12        all   1-12
lookall       colours only, no pick
kits          kit delivery         run   kits + 12 patients

---------------- COMPETITION ----------------
node          check start zone -> H CENTER (fix with leg1/leg2/leg3)
pre           -> ALL GOOD
Robot on the ORANGE start spot -> START button.

SLOT NUMBERS (robot faces the hospital):
       HOSPITAL
  [7][8][9]  |  [1][2][3]     hospital row
  [10][11][12] H [4][5][6]    start row
   LEFT side       RIGHT side      (nearest to H CENTER = smallest number)
