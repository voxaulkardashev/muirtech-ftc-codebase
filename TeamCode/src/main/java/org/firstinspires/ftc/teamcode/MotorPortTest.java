package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/*
 * Diagnostic: shows which hub + port each drive motor name is mapped to,
 * then spins each motor ONE AT A TIME for 2 seconds so you can see which
 * physical wheel (if any) responds to each name.
 *
 * Watch the "Current" reading while a motor runs:
 *   ~0.0 A  -> nothing is plugged into that port (config / wiring mismatch)
 *   > 0.1 A -> power is flowing; if the wrong wheel moved, the config is swapped
 *
 * Put the robot on a block before pressing START.
 */
@TeleOp(name = "Motor Port Test", group = "Test")
public class MotorPortTest extends LinearOpMode {

    private static final String[] NAMES = {
            "front_left_drive", "back_left_drive", "front_right_drive", "back_right_drive"
    };
    private static final double TEST_POWER   = 0.3;
    private static final double TEST_SECONDS = 2.0;

    @Override
    public void runOpMode() {
        DcMotorEx[] motors = new DcMotorEx[NAMES.length];
        for (int i = 0; i < NAMES.length; i++) {
            motors[i] = hardwareMap.get(DcMotorEx.class, NAMES[i]);
        }

        // During INIT: show where each name points. No movement yet.
        while (opModeInInit()) {
            telemetry.addLine("Config mapping (name -> hub / port):");
            for (int i = 0; i < NAMES.length; i++) {
                telemetry.addData(NAMES[i], motors[i].getConnectionInfo());
            }
            telemetry.addLine();
            telemetry.addLine("Press START to spin each motor one at a time.");
            telemetry.update();
        }

        for (int i = 0; i < NAMES.length && opModeIsActive(); i++) {
            DcMotorEx motor = motors[i];
            ElapsedTime timer = new ElapsedTime();
            double peakAmps = 0;

            motor.setPower(TEST_POWER);
            while (opModeIsActive() && timer.seconds() < TEST_SECONDS) {
                double amps = motor.getCurrent(CurrentUnit.AMPS);
                peakAmps = Math.max(peakAmps, amps);

                telemetry.addData("Now testing", NAMES[i]);
                telemetry.addData("Mapped to", motor.getConnectionInfo());
                telemetry.addData("Current", "%.2f A (peak %.2f A)", amps, peakAmps);
                telemetry.addData("Encoder", motor.getCurrentPosition());
                telemetry.update();
            }
            motor.setPower(0);
            sleep(500); // short pause so each wheel is clearly separate
        }

        telemetry.addLine("Test done. Note which wheels moved for which name.");
        telemetry.update();
        while (opModeIsActive()) {
            idle();
        }
    }
}
