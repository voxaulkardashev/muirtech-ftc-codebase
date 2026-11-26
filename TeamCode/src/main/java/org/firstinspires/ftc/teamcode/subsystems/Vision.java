package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/*
 * Wraps the Limelight 3A camera.
 *
 * Two jobs:
 *   1) AprilTag localization: "where is the robot on the field?" (getRobotPose)
 *      Uses MegaTag2, which needs the robot's heading each loop. Pass it from Odometry.
 *   2) Targeting: "where is the target relative to the camera?" (getTx / getTy / getTa)
 *      Works for any pipeline: AprilTags, color blobs, neural detector.
 *
 * Units: INCHES and DEGREES. Field coordinates match Odometry (origin = field center).
 *
 * Usage:
 *   Vision vision = new Vision(hardwareMap);           // INIT
 *   vision.start();
 *   loop { vision.update(odo.getHeading()); if (vision.hasRobotPose()) ... }
 *   vision.stop();                                     // end of OpMode
 *
 * Robot configuration: plug the Limelight into a Control Hub USB port, then Scan.
 * It shows up as an Ethernet device; name it "limelight".
 *
 * Limelight-side setup (done once from a laptop, in the Limelight web UI):
 *   - Pipeline 0 = AprilTag pipeline, with the current season's field map uploaded
 *     and "Full 3D" / botpose enabled.
 *   - Set the camera's mounting position on the robot (forward/side/up offsets and angle).
 *     botpose is only as accurate as these numbers.
 */
public class Vision {

    // ---------------------------------------------------------------------------------------
    // TUNE THESE to match how the Limelight is set up.
    // ---------------------------------------------------------------------------------------

    public static final String DEVICE_NAME = "limelight";

    // Pipeline slots as configured in the Limelight web UI.
    public static final int PIPELINE_APRILTAG = 0;
    public static final int PIPELINE_COLOR    = 1;  // placeholder for game-piece detection

    // Ignore results older than this. Old data = robot has moved since the picture.
    public static long MAX_STALENESS_MS = 100;

    // Ignore robot poses computed from tags farther than this (far tags are noisy).
    public static double MAX_TAG_DISTANCE_IN = 100;

    // ---------------------------------------------------------------------------------------

    private final Limelight3A limelight;
    private LLResult result = null;
    private int currentPipeline = PIPELINE_APRILTAG;

    public Vision(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, DEVICE_NAME);
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(PIPELINE_APRILTAG);
    }

    /** Begin polling. Without this, there is no data. */
    public void start() { limelight.start(); }

    /** Stop polling. Call when the OpMode ends. */
    public void stop()  { limelight.stop(); }

    public void switchPipeline(int index) {
        if (index != currentPipeline) {
            limelight.pipelineSwitch(index);
            currentPipeline = index;
        }
    }

    public int getPipeline() { return currentPipeline; }

    /**
     * Grab the newest result. Call ONCE per loop, after odometry.update().
     * @param robotHeadingDegrees field heading from Odometry (needed by MegaTag2)
     */
    public void update(double robotHeadingDegrees) {
        limelight.updateRobotOrientation(robotHeadingDegrees);
        result = limelight.getLatestResult();
    }

    // ----- Basic target info (any pipeline) -----

    /** True if the camera currently sees a target and the data is fresh. */
    public boolean hasTarget() {
        return result != null
                && result.isValid()
                && result.getStaleness() <= MAX_STALENESS_MS;
    }

    /** Horizontal angle to target, degrees. Positive = target is to the RIGHT of center. */
    public double getTx() { return hasTarget() ? result.getTx() : 0; }

    /** Vertical angle to target, degrees. Positive = target is ABOVE center. */
    public double getTy() { return hasTarget() ? result.getTy() : 0; }

    /** Target area, % of the image (0-100). Bigger = closer. */
    public double getTa() { return hasTarget() ? result.getTa() : 0; }

    // ----- AprilTags -----

    /** IDs of every AprilTag currently visible. Empty if none. */
    public List<Integer> getVisibleTagIds() {
        if (!hasTarget()) return Collections.emptyList();
        List<Integer> ids = new ArrayList<>();
        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            ids.add(tag.getFiducialId());
        }
        return ids;
    }

    /** The tag with the given ID if it's visible, otherwise null. */
    public LLResultTypes.FiducialResult getTag(int id) {
        if (!hasTarget()) return null;
        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            if (tag.getFiducialId() == id) return tag;
        }
        return null;
    }

    public boolean seesTag(int id) { return getTag(id) != null; }

    // ----- Robot position from AprilTags (MegaTag2) -----

    /** True if there's a trustworthy field position from AprilTags right now. */
    public boolean hasRobotPose() {
        if (!hasTarget()) return false;
        if (result.getBotposeTagCount() < 1) return false;

        double avgDistIn = DistanceUnit.METER.toInches(result.getBotposeAvgDist());
        if (avgDistIn > MAX_TAG_DISTANCE_IN) return false;

        Pose3D botpose = result.getBotpose_MT2();
        if (botpose == null) return false;
        Position p = botpose.getPosition();
        // An all-zero pose means the Limelight could not solve it.
        return !(p.x == 0 && p.y == 0);
    }

    /**
     * Robot's field position from AprilTags, in inches/degrees, or null if not trustworthy.
     * Heading is the one we fed in via update() (MegaTag2 trusts odometry for heading).
     */
    public Pose2D getRobotPose() {
        if (!hasRobotPose()) return null;
        Pose3D botpose = result.getBotpose_MT2();
        Position p = botpose.getPosition().toUnit(DistanceUnit.INCH);
        double heading = botpose.getOrientation().getYaw(AngleUnit.DEGREES);
        return new Pose2D(DistanceUnit.INCH, p.x, p.y, AngleUnit.DEGREES, heading);
    }

    /** How many tags the current robot pose was computed from (more = more accurate). */
    public int getPoseTagCount() { return hasTarget() ? result.getBotposeTagCount() : 0; }

    // ----- Health -----

    public boolean isConnected() { return limelight.isConnected(); }

    /** The raw result, for anything this class doesn't cover. May be null. */
    public LLResult getRawResult() { return result; }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("LL connected", isConnected());
        telemetry.addData("LL pipeline", currentPipeline);
        if (!hasTarget()) {
            telemetry.addData("LL target", "none");
            return;
        }
        telemetry.addData("LL target", "tx %.1f°, ty %.1f°, ta %.2f%%", getTx(), getTy(), getTa());
        telemetry.addData("LL tags", getVisibleTagIds());
        Pose2D pose = getRobotPose();
        if (pose != null) {
            telemetry.addData("LL pose", "X %.1f in, Y %.1f in (%d tags)",
                    pose.getX(DistanceUnit.INCH), pose.getY(DistanceUnit.INCH), getPoseTagCount());
        } else {
            telemetry.addData("LL pose", "not trustworthy");
        }
    }
}
