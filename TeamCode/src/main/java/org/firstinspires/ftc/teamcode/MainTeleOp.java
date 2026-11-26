package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Odometry;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

/*
 * Main driver-controlled program.
 *
 * GAMEPAD 1 (driver):
 *   Left stick      - drive forward/back and strafe left/right
 *   Right stick X   - turn
 *   Right bumper    - HOLD for slow mode (precise lining up)
 *   Y               - toggle field-centric / robot-centric   (needs working Pinpoint)
 *   Back / Share    - reset field-centric "forward" to the way the robot faces now
 *
 * GAMEPAD 2 (operator):
 *   Reserved for mechanisms (intake, lift, claw...) once they're built.
 *
 * REDUNDANCY: driving only REQUIRES the four drive motors.
 *   - Odometry (Pinpoint) missing from config, unplugged, or faulted -> automatically
 *     drives robot-centric. If it drops out mid-match, the driver's controller rumbles
 *     and driving continues robot-centric. If it comes back, field-centric resumes.
 *   - Vision (Limelight) is optional and not needed for driving; its status is shown
 *     so you know whether it's available for features like auto-aim later.
 */
@TeleOp(name = "Main TeleOp", group = "Competition")
public class MainTeleOp extends LinearOpMode {

    // Speed limits (0..1). Lower NORMAL_SPEED if the robot is too twitchy for new drivers.
    private static final double NORMAL_SPEED = 1.0;
    private static final double SLOW_SPEED   = 0.35;
    // Turning feels faster than driving, so scale it a little.
    private static final double TURN_SCALE   = 0.8;
    // Ignore tiny stick movements (stick drift).
    private static final double DEADZONE     = 0.05;

    private Odometry odometry = null;
    private Vision vision = null;
    private String odometryProblem = "";
    private String visionProblem = "";

    @Override
    public void runOpMode() {
        // The only REQUIRED hardware. If this fails, the motor config is wrong.
        MecanumDrive drive = new MecanumDrive(hardwareMap);

        // Optional hardware: any failure here just disables that feature.
/*         try {
            odometry = new Odometry(hardwareMap);
        } catch (Exception e) {
            odometry = null;
            odometryProblem = "not in robot config";
        }
        try {
            vision = new Vision(hardwareMap);
            vision.start();
        } catch (Exception e) {
            vision = null;
            visionProblem = "not in robot config";
        } */

        boolean fieldCentricWanted = true;   // driver's choice (Y button)
        boolean odometryWasWorking = false;  // for detecting a mid-match dropout
        double forwardHeadingOffset = 0;     // heading that counts as "away from the driver"

        // During INIT, keep showing hardware status so problems are visible before START.
        while (opModeInInit()) {
            boolean odoOk = updateOdometry();
            boolean visionOk = updateVision(odoOk ? odometry.getHeading() : 0);

            telemetry.addData("Status", "Initialized. Robot must be STILL until START.");
            telemetry.addData("Drive motors", "OK");
            telemetry.addData("Odometry", statusText(odoOk, odometry != null, odometryProblem,
                    "field-centric available"));
            telemetry.addData("Vision", statusText(visionOk, vision != null, visionProblem,
                    "available"));
            telemetry.addLine("Point the robot AWAY from the driver before pressing START.");
            telemetry.update();
        }

        if (updateOdometry()) {
            forwardHeadingOffset = odometry.getHeadingRadians();
            odometryWasWorking = true;
        }

        while (opModeIsActive()) {
            boolean odoOk = updateOdometry();
            boolean visionOk = updateVision(odoOk ? odometry.getHeading() : 0);

            // Warn the driver if odometry drops out mid-match.
            if (odometryWasWorking && !odoOk) {
                gamepad1.rumbleBlips(3);
            }
            odometryWasWorking = odoOk;

            // ---------------- Driver buttons ----------------
            if (gamepad1.yWasPressed()) {
                fieldCentricWanted = !fieldCentricWanted;
            }
            if (gamepad1.backWasPressed() && odoOk) {
                forwardHeadingOffset = odometry.getHeadingRadians();
                gamepad1.rumble(200);  // feedback so the driver knows it worked
            }

            // ---------------- Drive ----------------
            double speed = gamepad1.right_bumper ? SLOW_SPEED : NORMAL_SPEED;

            double forward = -deadzone(gamepad1.left_stick_y) * speed;  // stick up is negative
            double strafe  =  deadzone(gamepad1.left_stick_x) * speed;
            double turn    =  deadzone(gamepad1.right_stick_x) * TURN_SCALE * speed;

            boolean fieldCentric = fieldCentricWanted && odoOk;
            if (fieldCentric) {
                double heading = odometry.getHeadingRadians() - forwardHeadingOffset;
                drive.driveFieldCentric(forward, strafe, turn, heading);
            } else {
                drive.drive(forward, strafe, turn);
            }

            // ---------------- Mechanisms (gamepad 2) ----------------
            // TODO: add intake / lift / claw controls here once the hardware exists, e.g.
            //   intake.setPower(gamepad2.right_trigger - gamepad2.left_trigger);

            // ---------------- Telemetry ----------------
            String mode;
            if (fieldCentric)            mode = "FIELD-centric";
            else if (fieldCentricWanted) mode = "ROBOT-centric (odometry unavailable)";
            else                         mode = "ROBOT-centric";
            telemetry.addData("Mode", mode);
            telemetry.addData("Speed", gamepad1.right_bumper ? "SLOW" : "normal");
            telemetry.addData("Odometry", statusText(odoOk, odometry != null, odometryProblem, "OK"));
            telemetry.addData("Vision", statusText(visionOk, vision != null, visionProblem, "OK"));
            drive.addTelemetry(telemetry);
            if (odoOk) {
                odometry.addTelemetry(telemetry);
            }
            telemetry.update();
        }

        drive.stop();
        if (vision != null) {
            try { vision.stop(); } catch (Exception ignored) { }
        }
    }

    /** Update odometry; returns true only if it's present AND reporting healthy data. */
    private boolean updateOdometry() {
        if (odometry == null) return false;
        try {
            odometry.update();
            if (odometry.isReady()) {
                odometryProblem = "";
                return true;
            }
            odometryProblem = odometry.getStatus().toString();  // e.g. CALIBRATING, FAULT_NO_PODS_DETECTED
        } catch (Exception e) {
            odometryProblem = "read error";
        }
        return false;
    }

    /** Update vision; returns true only if it's present AND connected. */
    private boolean updateVision(double headingDegrees) {
        if (vision == null) return false;
        try {
            if (!vision.isConnected()) {
                visionProblem = "not connected";
                return false;
            }
            vision.update(headingDegrees);
            visionProblem = "";
            return true;
        } catch (Exception e) {
            visionProblem = "read error";
            return false;
        }
    }

    private static String statusText(boolean ok, boolean present, String problem, String okText) {
        if (ok) return okText;
        if (!present) return "MISSING (" + problem + ") - driving still works";
        return "PROBLEM (" + problem + ") - driving still works";
    }

    private static double deadzone(double value) {
        return Math.abs(value) < DEADZONE ? 0 : value;
    }
}
