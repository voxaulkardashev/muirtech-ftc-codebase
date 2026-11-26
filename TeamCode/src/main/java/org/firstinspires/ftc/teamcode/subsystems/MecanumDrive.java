package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
 * Four-motor mecanum drivetrain. Shared by TeleOp and Autonomous.
 *
 * Inputs are all -1..1:
 *   forward: + = drive forward
 *   strafe:  + = slide RIGHT
 *   turn:    + = rotate CLOCKWISE (right)
 *
 * Motor names must match the robot configuration:
 *   front_left_drive, back_left_drive, front_right_drive, back_right_drive
 */
public class MecanumDrive {

    /** Config names, in the same order as getMotors(). */
    public static final String[] MOTOR_NAMES = {
            "front_left_drive", "back_left_drive", "front_right_drive", "back_right_drive"
    };

    private final DcMotorEx frontLeft, backLeft, frontRight, backRight;

    private double lastFL, lastBL, lastFR, lastBR;

    public MecanumDrive(HardwareMap hardwareMap) {
        frontLeft  = hardwareMap.get(DcMotorEx.class, MOTOR_NAMES[0]);
        backLeft   = hardwareMap.get(DcMotorEx.class, MOTOR_NAMES[1]);
        frontRight = hardwareMap.get(DcMotorEx.class, MOTOR_NAMES[2]);
        backRight  = hardwareMap.get(DcMotorEx.class, MOTOR_NAMES[3]);

        // If a wheel spins backward when driving forward, flip FORWARD <-> REVERSE here.
        // Keep these in sync with StartupWheelSpin while testing.
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        for (DcMotorEx m : getMotors()) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            // Plain power control. Switch to RUN_USING_ENCODER later for smoother,
            // battery-independent speed (needs encoder cables plugged in).
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    /** Drive relative to the ROBOT: "forward" means the way the robot is facing. */
    public void drive(double forward, double strafe, double turn) {
        double fl = forward + strafe + turn;
        double fr = forward - strafe - turn;
        double bl = forward - strafe + turn;
        double br = forward + strafe - turn;

        // Scale down so no wheel goes above 100% (keeps the direction of motion correct).
        double max = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                                            Math.max(Math.abs(bl), Math.abs(br))));
        setPowers(fl / max, bl / max, fr / max, br / max);
    }

    /**
     * Drive relative to the FIELD: "forward" means away from the driver, no matter
     * which way the robot is facing.
     * @param headingRadians robot heading (counter-clockwise positive), e.g. Odometry.getHeadingRadians()
     */
    public void driveFieldCentric(double forward, double strafe, double turn, double headingRadians) {
        // Rotate the joystick vector by -heading to get robot-relative motion.
        // (strafe is +right, heading is +counter-clockwise.)
        // Sanity check: robot turned 90° left, driver pushes forward -> robot slides to ITS right.
        double cos = Math.cos(headingRadians);
        double sin = Math.sin(headingRadians);
        double robotForward = forward * cos - strafe * sin;
        double robotStrafe  = strafe * cos + forward * sin;
        drive(robotForward, robotStrafe, turn);
    }

    public void stop() {
        setPowers(0, 0, 0, 0);
    }

    /** The four motors (FL, BL, FR, BR), for diagnostics like current draw. */
    public DcMotorEx[] getMotors() {
        return new DcMotorEx[]{frontLeft, backLeft, frontRight, backRight};
    }

    private void setPowers(double fl, double bl, double fr, double br) {
        frontLeft.setPower(fl);
        backLeft.setPower(bl);
        frontRight.setPower(fr);
        backRight.setPower(br);
        lastFL = fl; lastBL = bl; lastFR = fr; lastBR = br;
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Drive front L/R", "%5.2f  %5.2f", lastFL, lastFR);
        telemetry.addData("Drive back  L/R", "%5.2f  %5.2f", lastBL, lastBR);
    }
}
