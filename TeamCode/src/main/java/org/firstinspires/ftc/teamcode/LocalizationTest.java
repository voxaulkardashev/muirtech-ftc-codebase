package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.Odometry;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

/*
 * Test Odometry (Pinpoint) and Vision (Limelight) together. No motors are used,
 * so push the robot around by hand and watch the numbers.
 *
 * Controls (gamepad 1):
 *   A  - reset odometry to (0, 0, 0°)
 *   B  - snap odometry to the Limelight's AprilTag position (if it sees a tag)
 *   X  - toggle AUTO-CORRECT: odometry is snapped to vision whenever the robot
 *        is nearly still and a trustworthy tag position is available
 *
 * Checks to do:
 *   1) Push robot forward 24 in: odometry X should go up by ~24.
 *   2) Push robot left 24 in:    odometry Y should go up by ~24.
 *   3) Spin robot left 90°:      heading should go up by ~90.
 *   4) Point the Limelight at a tag: vision pose should roughly match odometry
 *      (once odometry has been snapped to vision with B).
 */
@TeleOp(name = "Localization Test", group = "Test")
public class LocalizationTest extends LinearOpMode {

    // Only auto-correct when the robot is moving slower than this (motion blur = bad poses).
    private static final double AUTO_CORRECT_MAX_SPEED_IN_PER_S = 5.0;

    @Override
    public void runOpMode() {
        Odometry odometry = new Odometry(hardwareMap);
        Vision vision = new Vision(hardwareMap);
        vision.start();

        boolean autoCorrect = false;
        boolean prevA = false, prevB = false, prevX = false;
        int corrections = 0;

        telemetry.setMsTransmissionInterval(50);

        // Show data during INIT too, so you can test without pressing START.
        while (!isStopRequested()) {
            odometry.update();
            vision.update(odometry.getHeading());

            // Button presses (trigger once per press, not every loop).
            boolean a = gamepad1.a, b = gamepad1.b, x = gamepad1.x;

            if (a && !prevA) {
                odometry.setPose(0, 0, 0);
            }
            if (b && !prevB) {
                Pose2D visionPose = vision.getRobotPose();
                if (visionPose != null) {
                    // Keep our own heading: MegaTag2 gets heading from us anyway.
                    odometry.setPose(visionPose.getX(DistanceUnit.INCH),
                                     visionPose.getY(DistanceUnit.INCH),
                                     odometry.getHeading());
                    corrections++;
                }
            }
            if (x && !prevX) {
                autoCorrect = !autoCorrect;
            }
            prevA = a; prevB = b; prevX = x;

            if (autoCorrect && odometry.getSpeed() < AUTO_CORRECT_MAX_SPEED_IN_PER_S) {
                Pose2D visionPose = vision.getRobotPose();
                if (visionPose != null) {
                    odometry.setPose(visionPose.getX(DistanceUnit.INCH),
                                     visionPose.getY(DistanceUnit.INCH),
                                     odometry.getHeading());
                    corrections++;
                }
            }

            telemetry.addData("Mode", opModeIsActive() ? "RUNNING" : "INIT (data is live)");
            telemetry.addLine("A = zero   B = snap to vision   X = auto-correct");
            telemetry.addData("Auto-correct", autoCorrect ? "ON" : "off");
            telemetry.addData("Corrections applied", corrections);
            telemetry.addLine();
            odometry.addTelemetry(telemetry);
            telemetry.addData("Odo speed", "%.1f in/s", odometry.getSpeed());
            telemetry.addLine();
            vision.addTelemetry(telemetry);
            telemetry.update();
        }

        vision.stop();
    }
}
