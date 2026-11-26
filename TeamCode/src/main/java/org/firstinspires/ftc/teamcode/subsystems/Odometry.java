package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

/*
 * Tracks where the robot is on the field using the goBILDA Pinpoint odometry computer.
 *
 * Units used everywhere in this class: INCHES and DEGREES.
 *
 * Coordinate system (matches the FTC field coordinate system and the Limelight's botpose,
 * so vision can correct odometry directly):
 *   - Origin (0, 0) is the CENTER of the field.
 *   - Heading is counter-clockwise positive.
 *
 * Usage:
 *   Odometry odo = new Odometry(hardwareMap);   // in INIT, robot standing still
 *   odo.setPose(startX, startY, startHeading);  // where the robot starts on the field
 *   loop { odo.update(); double x = odo.getX(); ... }
 *
 * Robot configuration: add the Pinpoint on an I2C bus as
 *   "goBILDA Pinpoint Odometry Computer" named "pinpoint".
 */
public class Odometry {

    // ---------------------------------------------------------------------------------------
    // TUNE THESE once the pods are mounted on the robot.
    // ---------------------------------------------------------------------------------------

    public static final String DEVICE_NAME = "pinpoint";

    // How far SIDEWAYS the X (forward-tracking) pod is from the robot's center.
    // Left of center = positive, right of center = negative.
    public static double X_POD_OFFSET_MM = 0.0;

    // How far FORWARD the Y (strafe-tracking) pod is from the robot's center.
    // Forward of center = positive, behind center = negative.
    public static double Y_POD_OFFSET_MM = 0.0;

    // goBILDA_4_BAR_POD or goBILDA_SWINGARM_POD (check which pods your team bought).
    public static GoBildaPinpointDriver.GoBildaOdometryPods POD_TYPE =
            GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;

    // Push the robot forward: X should go UP. Push it left: Y should go UP.
    // If either goes the wrong way, flip it to REVERSED.
    public static GoBildaPinpointDriver.EncoderDirection X_POD_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static GoBildaPinpointDriver.EncoderDirection Y_POD_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;

    // ---------------------------------------------------------------------------------------

    private final GoBildaPinpointDriver pinpoint;
    private Pose2D pose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);

    /** Call during INIT with the robot completely still (the IMU calibrates here). */
    public Odometry(HardwareMap hardwareMap) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, DEVICE_NAME);

        pinpoint.setOffsets(X_POD_OFFSET_MM, Y_POD_OFFSET_MM, DistanceUnit.MM);
        pinpoint.setEncoderResolution(POD_TYPE);
        pinpoint.setEncoderDirections(X_POD_DIRECTION, Y_POD_DIRECTION);

        // Zero the position and recalibrate the IMU. Takes ~0.25 s; robot must not move.
        pinpoint.resetPosAndIMU();
    }

    /** Read the latest data from the Pinpoint. Call this ONCE at the top of every loop. */
    public void update() {
        pinpoint.update();
        pose = pinpoint.getPosition();
    }

    /** Tell the Pinpoint where the robot is (start of auto, or a correction from vision). */
    public void setPose(double xInches, double yInches, double headingDegrees) {
        Pose2D newPose = new Pose2D(DistanceUnit.INCH, xInches, yInches, AngleUnit.DEGREES, headingDegrees);
        pinpoint.setPosition(newPose);
        pose = newPose;
    }

    public void setPose(Pose2D newPose) {
        setPose(newPose.getX(DistanceUnit.INCH),
                newPose.getY(DistanceUnit.INCH),
                newPose.getHeading(AngleUnit.DEGREES));
    }

    /** Reset heading only, e.g. a "reset field-centric" button in TeleOp. */
    public void setHeading(double headingDegrees) {
        pinpoint.setHeading(headingDegrees, AngleUnit.DEGREES);
    }

    // ----- Position (from the last update()) -----

    public Pose2D getPose()         { return pose; }
    public double getX()            { return pose.getX(DistanceUnit.INCH); }
    public double getY()            { return pose.getY(DistanceUnit.INCH); }
    /** Heading in degrees, wrapped to -180..180. */
    public double getHeading()      { return pose.getHeading(AngleUnit.DEGREES); }
    public double getHeadingRadians() { return pose.getHeading(AngleUnit.RADIANS); }

    // ----- Velocity (field frame) -----

    public double getVelX()         { return pinpoint.getVelX(DistanceUnit.INCH); }
    public double getVelY()         { return pinpoint.getVelY(DistanceUnit.INCH); }
    /** Speed in inches/second regardless of direction. */
    public double getSpeed()        { return Math.hypot(getVelX(), getVelY()); }
    public double getTurnRate()     { return pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES); }

    // ----- Health -----

    public GoBildaPinpointDriver.DeviceStatus getStatus() { return pinpoint.getDeviceStatus(); }
    public boolean isReady() { return getStatus() == GoBildaPinpointDriver.DeviceStatus.READY; }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Odo status", getStatus());
        telemetry.addData("Odo pose", "X %.1f in, Y %.1f in, H %.1f°", getX(), getY(), getHeading());
        telemetry.addData("Odo raw ticks", "X %d, Y %d", pinpoint.getEncoderX(), pinpoint.getEncoderY());
    }
}
