package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * Startup check: when the driver presses START, all four drive wheels spin
 * forward for 2 seconds, then stop. Use it to confirm the robot is alive and
 * that every motor is wired, configured, and turning the right way.
 *
 * NOTE: Wheels on the ground means the robot WILL drive forward for 2 seconds.
 *       Put the robot up on a block, or give it room.
 *
 * Hardware config names (must match the robot configuration on the Driver Station):
 *   front_left_drive, back_left_drive, front_right_drive, back_right_drive
 */
@TeleOp(name = "Startup Wheel Spin", group = "Test")
public class StartupWheelSpin extends LinearOpMode {

    private static final double SPIN_POWER   = 0.3;  // 30% power, gentle
    private static final double SPIN_SECONDS = 2.0;

    @Override
    public void runOpMode() {
        DcMotor frontLeftDrive  = hardwareMap.get(DcMotor.class, "front_left_drive");
        DcMotor backLeftDrive   = hardwareMap.get(DcMotor.class, "back_left_drive");
        DcMotor frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        DcMotor backRightDrive  = hardwareMap.get(DcMotor.class, "back_right_drive");

        // Most drivetrains need one side reversed so all wheels push the robot forward.
        // If a wheel spins backward during this test, flip its FORWARD <-> REVERSE here.
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Brake (not coast) when power goes to 0, so the stop is crisp.
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized. Press START to spin wheels.");
        telemetry.update();

        // Motors must not move during INIT (game rule), so wait for START.
        waitForStart();

        ElapsedTime timer = new ElapsedTime();

        frontLeftDrive.setPower(SPIN_POWER);
        backLeftDrive.setPower(SPIN_POWER);
        frontRightDrive.setPower(SPIN_POWER);
        backRightDrive.setPower(SPIN_POWER);

        // Loop instead of sleep() so pressing STOP ends it immediately.
        while (opModeIsActive() && timer.seconds() < SPIN_SECONDS) {
            telemetry.addData("Status", "Spinning wheels");
            telemetry.addData("Time", "%.1f / %.1f s", timer.seconds(), SPIN_SECONDS);
            telemetry.update();
        }

        frontLeftDrive.setPower(0);
        backLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backRightDrive.setPower(0);

        telemetry.addData("Status", "Startup complete, wheels stopped.");
        telemetry.update();

        // Hold here (motors off) until the driver presses STOP.
        while (opModeIsActive()) {
            idle();
        }
    }
}
