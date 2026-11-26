package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Odometry;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

/*
 * STRESS TEST: rapidly reverses the drivetrain at high power while recording how every
 * component holds up: motor current, encoders, battery voltage, Pinpoint and Limelight
 * connection, and Control Hub loop time. Ends with a report.
 *
 * !!! SAFETY !!!
 *   - Hard reversals at full power are rough on gearboxes, wheels and the battery.
 *     That's the point, but don't run it longer than needed, and let motors cool between runs.
 *   - Clear the area. On the ground the robot will lurch back and forth.
 *     Up on a block tests motors/wiring only (not traction or chassis flex).
 *   - DEAD-MAN SWITCH: motors only run while gamepad1 RIGHT BUMPER is HELD.
 *     Let go and everything stops instantly. Press B to end the test early.
 *   - Auto-abort if any motor draws too much current for too long (stall / jam)
 *     or the battery sags too low.
 *
 * INIT menu (gamepad1): D-pad up/down picks a setting, left/right changes it.
 */
@TeleOp(name = "Stress Test", group = "Test")
public class StressTest extends LinearOpMode {

    // ---------------- Safety limits ----------------
    private static final double STALL_AMPS        = 8.0;   // per motor
    private static final double STALL_TIME_MS     = 500;   // this long above STALL_AMPS = abort
    private static final double LOW_BATTERY_VOLTS = 9.0;
    private static final double LOW_BATTERY_MS    = 1000;  // this long below = abort
    private static final double RECOVERY_SECONDS  = 3.0;   // measure bounce-back after the test

    // ---------------- Menu options ----------------
    private enum Pattern { FORWARD_BACK, STRAFE, SPIN, MIXED }
    private static final Pattern[] PATTERNS   = Pattern.values();
    private static final double[]  POWERS     = {0.5, 0.75, 1.0};
    private static final int[]     INTERVALS_MS = {150, 250, 500, 1000};
    private static final int[]     DURATIONS_S  = {10, 30, 60, 120};
    private static final String[]  MENU_ROWS  = {"Pattern", "Power", "Reverse every", "Duration"};

    private int patternIdx = 0, powerIdx = 2, intervalIdx = 1, durationIdx = 1;
    private int menuRow = 0;

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap);
        DcMotorEx[] motors = drive.getMotors();
        String[] names = MecanumDrive.MOTOR_NAMES;
        int n = motors.length;

        // Bulk reads: one hub read per loop instead of one per sensor call.
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Optional components: tested if present, ignored if not.
        Odometry odometry = null;
        Vision vision = null;
        try { odometry = new Odometry(hardwareMap); } catch (Exception ignored) { }
        try { vision = new Vision(hardwareMap); vision.start(); } catch (Exception ignored) { }

        // ================= INIT: settings menu =================
        // Resting battery voltage (motors off), smoothed so one noisy reading doesn't skew it.
        double restingVolts = 0;
        while (opModeInInit()) {
            double v = batteryVolts();
            if (v > 0) restingVolts = (restingVolts == 0) ? v : restingVolts * 0.9 + v * 0.1;

            if (gamepad1.dpadUpWasPressed())   menuRow = (menuRow + MENU_ROWS.length - 1) % MENU_ROWS.length;
            if (gamepad1.dpadDownWasPressed()) menuRow = (menuRow + 1) % MENU_ROWS.length;
            int change = (gamepad1.dpadRightWasPressed() ? 1 : 0) - (gamepad1.dpadLeftWasPressed() ? 1 : 0);
            if (change != 0) {
                switch (menuRow) {
                    case 0: patternIdx  = wrap(patternIdx + change, PATTERNS.length); break;
                    case 1: powerIdx    = wrap(powerIdx + change, POWERS.length); break;
                    case 2: intervalIdx = wrap(intervalIdx + change, INTERVALS_MS.length); break;
                    case 3: durationIdx = wrap(durationIdx + change, DURATIONS_S.length); break;
                }
            }

            telemetry.addLine("=== STRESS TEST SETUP ===");
            telemetry.addLine("D-pad up/down: select   left/right: change");
            String[] values = {
                    PATTERNS[patternIdx].toString(),
                    (int) (POWERS[powerIdx] * 100) + "%",
                    INTERVALS_MS[intervalIdx] + " ms",
                    DURATIONS_S[durationIdx] + " s"
            };
            for (int i = 0; i < MENU_ROWS.length; i++) {
                telemetry.addLine((i == menuRow ? "> " : "   ") + MENU_ROWS[i] + ": " + values[i]);
            }
            telemetry.addLine();
            telemetry.addData("Battery (resting)", "%.2f V", restingVolts);
            telemetry.addData("Odometry", odometry != null ? "will be monitored" : "not configured (skipped)");
            telemetry.addData("Vision",   vision   != null ? "will be monitored" : "not configured (skipped)");
            telemetry.addLine();
            telemetry.addLine("!! Clear the area. Robot will lurch at high speed.");
            telemetry.addLine("After START: HOLD RIGHT BUMPER to run. Release = stop. B = end.");
            telemetry.update();
        }

        Pattern pattern   = PATTERNS[patternIdx];
        double power      = POWERS[powerIdx];
        int intervalMs    = INTERVALS_MS[intervalIdx];
        double durationMs = DURATIONS_S[durationIdx] * 1000.0;

        // ================= Stats =================
        double[] nowAmps  = new double[n];
        double[] peakAmps = new double[n];
        double[] sumAmps  = new double[n];
        long[]   travel   = new long[n];
        int[]    lastPos  = new int[n];
        double[] stallSince = new double[n];
        for (int i = 0; i < n; i++) {
            lastPos[i] = motors[i].getCurrentPosition();
            stallSince[i] = -1;
        }
        long ampSamples = 0;

        double minVolts = Double.MAX_VALUE;
        double lowBattSince = -1;
        double sumLoadedVolts = 0;
        long loadedVoltSamples = 0;
        double peakTotalAmps = 0, voltsAtPeakAmps = 0;
        boolean batteryFailed = false;
        int reversals = 0;
        long lastPhase = -1;
        int odoDropouts = 0, visionDropouts = 0;
        boolean odoWasOk = odometry != null && safeOdoOk(odometry);
        boolean visionWasOk = vision != null && safeVisionOk(vision);
        double maxLoopMs = 0;
        long loops = 0;

        double activeMs = 0;            // time spent actually running (bumper held)
        String endReason = "Completed full duration";

        ElapsedTime clock = new ElapsedTime();
        ElapsedTime loopTimer = new ElapsedTime();

        // ================= Test loop =================
        while (opModeIsActive()) {
            double dt = loopTimer.milliseconds();
            loopTimer.reset();
            if (loops > 0) maxLoopMs = Math.max(maxLoopMs, dt);
            loops++;
            double now = clock.milliseconds();

            for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) hub.clearBulkCache();

            if (gamepad1.b) { endReason = "Ended early by driver (B)"; break; }
            if (activeMs >= durationMs) break;

            // ----- Drive pattern (only while dead-man is held) -----
            boolean armed = gamepad1.right_bumper;
            if (armed) {
                activeMs += dt;
                long phase = (long) (activeMs / intervalMs);
                if (lastPhase >= 0 && phase != lastPhase) reversals++;
                lastPhase = phase;

                double s = (phase % 2 == 0 ? 1 : -1) * power;
                Pattern p = pattern;
                if (p == Pattern.MIXED) {
                    // Cycle forward/back -> strafe -> spin, one full back-and-forth each.
                    p = PATTERNS[(int) ((phase / 2) % 3)];
                }
                switch (p) {
                    case FORWARD_BACK: drive.drive(s, 0, 0); break;
                    case STRAFE:       drive.drive(0, s, 0); break;
                    default:           drive.drive(0, 0, s); break;
                }
            } else {
                drive.stop();
            }

            // ----- Motors: current + encoders -----
            String abort = null;
            for (int i = 0; i < n; i++) {
                double amps = motors[i].getCurrent(CurrentUnit.AMPS);
                nowAmps[i] = amps;
                peakAmps[i] = Math.max(peakAmps[i], amps);
                if (armed) sumAmps[i] += amps;

                int pos = motors[i].getCurrentPosition();
                travel[i] += Math.abs(pos - lastPos[i]);
                lastPos[i] = pos;

                if (armed && amps > STALL_AMPS) {
                    if (stallSince[i] < 0) stallSince[i] = now;
                    else if (now - stallSince[i] > STALL_TIME_MS) {
                        abort = String.format("ABORT: %s over %.0f A for %.1f s (jam/stall?)",
                                names[i], STALL_AMPS, STALL_TIME_MS / 1000);
                    }
                } else {
                    stallSince[i] = -1;
                }
            }
            if (armed) ampSamples++;

            // ----- Battery -----
            double volts = batteryVolts();
            if (volts > 0) {
                minVolts = Math.min(minVolts, volts);
                if (armed) {
                    sumLoadedVolts += volts;
                    loadedVoltSamples++;
                    double totalAmps = 0;
                    for (double a : nowAmps) totalAmps += a;
                    if (totalAmps > peakTotalAmps) {
                        peakTotalAmps = totalAmps;
                        voltsAtPeakAmps = volts;
                    }
                }
                if (volts < LOW_BATTERY_VOLTS) {
                    if (lowBattSince < 0) lowBattSince = now;
                    else if (now - lowBattSince > LOW_BATTERY_MS) {
                        batteryFailed = true;
                        abort = String.format("BATTERY FAILED: stayed below %.1f V under load", LOW_BATTERY_VOLTS);
                    }
                } else {
                    lowBattSince = -1;
                }
            }

            if (abort != null) { endReason = abort; break; }

            // ----- Odometry / vision connection under vibration -----
            boolean odoOk = false, visionOk = false;
            if (odometry != null) {
                odoOk = safeOdoOk(odometry);
                if (odoWasOk && !odoOk) odoDropouts++;
                odoWasOk = odoOk;
            }
            if (vision != null) {
                visionOk = safeVisionOk(vision);
                if (visionWasOk && !visionOk) visionDropouts++;
                visionWasOk = visionOk;
            }

            // ----- Live telemetry -----
            telemetry.addData("State", armed ? "RUNNING" : "PAUSED (hold right bumper)");
            telemetry.addData("Test", "%s @ %d%%, reverse every %d ms", pattern, (int) (power * 100), intervalMs);
            telemetry.addData("Progress", "%.1f / %.0f s, %d reversals", activeMs / 1000, durationMs / 1000, reversals);
            telemetry.addData("Battery", "%.2f V (min %.2f)", volts, minVolts);
            for (int i = 0; i < n; i++) {
                telemetry.addData(names[i], "%.2f A (peak %.2f)", nowAmps[i], peakAmps[i]);
            }
            if (odometry != null) telemetry.addData("Odometry", odoOk ? "OK" : "PROBLEM: " + odometry.getStatus());
            if (vision != null)   telemetry.addData("Vision", visionOk ? "OK" : "DISCONNECTED");
            telemetry.addData("Loop", "%.0f ms (max %.0f)", dt, maxLoopMs);
            telemetry.addLine("Release bumper = stop.  B = end test.");
            telemetry.update();
        }

        drive.stop();

        // ================= Battery recovery =================
        // A healthy battery bounces back close to its resting voltage within a few seconds.
        double recoveredVolts = 0;
        ElapsedTime recovery = new ElapsedTime();
        while (opModeIsActive() && recovery.seconds() < RECOVERY_SECONDS) {
            recoveredVolts = batteryVolts();
            telemetry.addLine("Test finished. Motors OFF.");
            telemetry.addData("Measuring battery recovery", "%.1f / %.0f s  (%.2f V)",
                    recovery.seconds(), RECOVERY_SECONDS, recoveredVolts);
            telemetry.update();
        }

        double avgLoadedVolts = loadedVoltSamples > 0 ? sumLoadedVolts / loadedVoltSamples : 0;
        double sag = (restingVolts > 0 && minVolts != Double.MAX_VALUE) ? restingVolts - minVolts : 0;
        // Internal resistance estimate (battery + wiring + connectors): voltage drop per amp drawn.
        double resistanceMilliOhms = (peakTotalAmps > 1.0 && restingVolts > 0)
                ? (restingVolts - voltsAtPeakAmps) / peakTotalAmps * 1000 : 0;
        String batteryVerdict = batteryVerdict(batteryFailed, minVolts, loadedVoltSamples > 0);

        // ================= Report =================
        double meanAvg = 0;
        double[] avgAmps = new double[n];
        for (int i = 0; i < n; i++) {
            avgAmps[i] = ampSamples > 0 ? sumAmps[i] / ampSamples : 0;
            meanAvg += avgAmps[i] / n;
        }

        while (opModeIsActive()) {
            telemetry.addLine("=== STRESS TEST REPORT ===");
            telemetry.addData("Result", endReason);
            telemetry.addData("Ran", "%.1f s, %d reversals, %s @ %d%%",
                    activeMs / 1000, reversals, pattern, (int) (power * 100));
            telemetry.addLine();
            telemetry.addLine("--- BATTERY ---");
            telemetry.addData("Verdict", batteryVerdict);
            telemetry.addData("Resting (before)", "%.2f V", restingVolts);
            telemetry.addData("Under load", "avg %.2f V, lowest %.2f V",
                    avgLoadedVolts, minVolts == Double.MAX_VALUE ? 0 : minVolts);
            telemetry.addData("Sag", "%.2f V", sag);
            telemetry.addData("Peak total draw", "%.1f A (at %.2f V)", peakTotalAmps, voltsAtPeakAmps);
            telemetry.addData("Est. resistance", resistanceMilliOhms > 0
                    ? String.format("%.0f mΩ (lower = healthier)", resistanceMilliOhms) : "n/a");
            telemetry.addData("Recovered (after %.0f s)", "%.2f V", RECOVERY_SECONDS, recoveredVolts);
            telemetry.addLine();
            telemetry.addLine("--- MOTORS (did power get through?) ---");
            for (int i = 0; i < n; i++) {
                telemetry.addData(names[i], "avg %.2f A, peak %.2f A, enc %d", avgAmps[i], peakAmps[i], travel[i]);
                telemetry.addData("  verdict", motorVerdict(avgAmps[i], peakAmps[i], travel[i], meanAvg, ampSamples > 0));
            }
            telemetry.addLine();
            telemetry.addData("Odometry", odometry == null ? "not tested"
                    : odoDropouts == 0 ? "OK, no dropouts" : odoDropouts + " DROPOUT(S): check Pinpoint cable");
            telemetry.addData("Vision", vision == null ? "not tested"
                    : visionDropouts == 0 ? "OK, no dropouts" : visionDropouts + " DROPOUT(S): check Limelight USB");
            telemetry.addData("Max loop time", "%.0f ms %s", maxLoopMs, maxLoopMs > 100 ? "(SLOW: Control Hub lagging)" : "");
            telemetry.addLine();
            telemetry.addLine("Press STOP to exit. Let motors cool before another run.");
            telemetry.update();
        }

        if (vision != null) {
            try { vision.stop(); } catch (Exception ignored) { }
        }
    }

    /**
     * Rough rule of thumb for a 12 V FTC NiMH pack under heavy drivetrain load.
     * Best used to COMPARE your batteries against each other with the same test settings.
     */
    private static String batteryVerdict(boolean failed, double minVolts, boolean ran) {
        if (!ran) return "not tested (bumper never held)";
        if (failed) return "FAILED: couldn't hold voltage. Recharge; if it repeats, retire it";
        if (minVolts >= 11.0) return "STRONG: good for matches";
        if (minVolts >= 10.0) return "OK: fine for matches";
        if (minVolts >= LOW_BATTERY_VOLTS) return "WEAK: practice only, or recharge and retest";
        return "WEAK: dipped low briefly. Recharge and retest";
    }

    /** A plain-English judgement of one motor's results. */
    private static String motorVerdict(double avg, double peak, long travel, double meanAvg, boolean ran) {
        if (!ran) return "not run (bumper never held)";
        if (peak < 0.2) return "NO CURRENT: unplugged or wrong port?";
        StringBuilder sb = new StringBuilder();
        if (meanAvg > 0.1 && avg > meanAvg * 1.5) sb.append("HIGH draw vs others: friction/binding? ");
        if (meanAvg > 0.1 && avg < meanAvg * 0.5) sb.append("LOW draw vs others: slipping/loose? ");
        if (travel == 0) sb.append("no encoder counts (cable unplugged?) ");
        return sb.length() == 0 ? "OK" : sb.toString().trim();
    }

    private double batteryVolts() {
        double lowest = Double.MAX_VALUE;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double v = sensor.getVoltage();
            if (v > 0) lowest = Math.min(lowest, v);
        }
        return lowest == Double.MAX_VALUE ? 0 : lowest;
    }

    private static boolean safeOdoOk(Odometry odometry) {
        try {
            odometry.update();
            return odometry.isReady();
        } catch (Exception e) {
            return false;
        }
    }

    private static boolean safeVisionOk(Vision vision) {
        try {
            return vision.isConnected();
        } catch (Exception e) {
            return false;
        }
    }

    private static int wrap(int value, int size) {
        return ((value % size) + size) % size;
    }
}
