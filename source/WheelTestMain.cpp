// =====================================================================
// WheelTestMain.cpp — TEMPORARY per-wheel motor diagnostic (046-008).
//
// A throwaway ALTERNATIVE main: drives each mecanum wheel directly via the
// HAL's Motor objects (Motor::setSpeed -> fwdSign -> Nezha I2C), bypassing the
// ENTIRE production control stack (MotionController / BodyVelocityController /
// kinematics / MotorController / Drive loop). Lets us verify per-wheel
// direction/response AND multi-wheel combinations with no control loop fighting
// the motors.
//
// Reads command lines from BOTH transports via the production Communicator:
//   * radio  (comm.radio().poll)   — the radio RELAY (untethered)
//   * serial (comm.serial().readLine) — USB-direct
// so the SAME host works tethered or over the relay.
//
// LINE protocol: each line lists the wheels to drive; listed wheels run, the
// rest stop. A line with NO wheel keys (e.g. the relay "+" keepalive) is IGNORED
// — it must not disturb the drive. Release/host-loss -> deadman stops in ~0.5 s.
//     Q/W = FL fwd/back   E/R = FR fwd/back   Z/X = BL fwd/back   C/V = BR fwd/back
//   e.g. "QEZC" = all forward;  "QRCX" = strafe-left pattern.
//
// "fwd" drives the wheel at +speed (logical forward); the Motor layer applies
// the per-wheel fwdSign, so this also validates the configured signs.
//
// Wedged in from main.cpp under `#if WHEEL_TEST_MAIN`. To restore normal boot:
// delete this file and that block (set WHEEL_TEST_MAIN 0). Mecanum-only.
// =====================================================================
#ifdef ROBOT_DRIVETRAIN_MECANUM
#include "MicroBit.h"
#include "MecanumHAL.h"
#include "Communicator.h"

// Parse one line of wheel keys. Returns true iff it held >=1 recognized wheel
// key (and updated fl/fr/bl/br). Blank/keepalive lines return false and leave
// the drive untouched.
static bool parseWheels(const char* s, int8_t SPD,
                        int8_t& fl, int8_t& fr, int8_t& bl, int8_t& br) {
    int8_t a = 0, b = 0, d = 0, e = 0;
    bool any = false;
    for (const char* p = s; *p; ++p) {
        switch (*p) {
            case 'q': case 'Q': a = +SPD; any = true; break;
            case 'w': case 'W': a = -SPD; any = true; break;
            case 'e': case 'E': b = +SPD; any = true; break;
            case 'r': case 'R': b = -SPD; any = true; break;
            case 'z': case 'Z': d = +SPD; any = true; break;
            case 'x': case 'X': d = -SPD; any = true; break;
            case 'c': case 'C': e = +SPD; any = true; break;
            case 'v': case 'V': e = -SPD; any = true; break;
            default: break;
        }
    }
    if (any) { fl = a; fr = b; bl = d; br = e; }
    return any;
}

void wheelTestMain(MicroBit& uBit, MecanumHAL& hw, Communicator& comm) {
    const int8_t SPD = 35;   // percent — gentle, just enough to see motion
    comm.serial().send("WHEELTEST 046-008 (serial+radio) ready\r\n");

    int8_t fl = 0, fr = 0, bl = 0, br = 0;
    uint32_t lastWheelMs = uBit.systemTime();
    char buf[80];

    while (true) {
        // Pull command lines from BOTH transports (relay radio + USB serial).
        if (comm.radio().poll(buf, sizeof(buf))) {
            if (parseWheels(buf, SPD, fl, fr, bl, br)) lastWheelMs = uBit.systemTime();
        }
        if (comm.serial().readLine(buf, sizeof(buf))) {
            if (parseWheels(buf, SPD, fl, fr, bl, br)) lastWheelMs = uBit.systemTime();
        }

        // Deadman: stop ~500 ms after the last wheel line (keys released / host gone).
        if (uBit.systemTime() - lastWheelMs > 500) { fl = fr = bl = br = 0; }

        hw.motorL().setSpeed(fl);    // FL (front-left)
        hw.motorR().setSpeed(fr);    // FR (front-right)
        hw.motorBL().setSpeed(bl);   // BL (back-left)
        hw.motorBR().setSpeed(br);   // BR (back-right)

        uBit.sleep(20);
    }
}
#endif  // ROBOT_DRIVETRAIN_MECANUM
