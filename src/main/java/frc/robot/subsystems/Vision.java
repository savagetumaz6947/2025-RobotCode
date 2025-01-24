package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class Vision {
    private PhotonCamera camera;
    private PhotonPoseEstimator photonEstimator;

    /**
     * Constructs a Vision object with no transformation (primarily for use with Object Detection)
     * @param cameraName The name of the PhotonCamera
     */
    public Vision(String cameraName) {
        this(cameraName, new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0)));
    }

    /**
     * Constructs a Vision object with the predefined field layout
     * @param cameraName The name of the PhotonCamera
     * @param robotToCam The location of the camera relative to the robot center
     */
    public Vision(String cameraName, Transform3d robotToCam) {
        this(cameraName, robotToCam, AprilTagFields.k2024Crescendo.loadAprilTagLayoutField());
    }

    /**
     * Constructs a Vision object with custom AprilTagFieldLayout
     * @param cameraName The name of the PhotonCamera
     * @param robotToCam The location of the camera relative to the robot center
     * @param aprilTagFieldLayout The custom AprilTagFieldLayout
     */
    public Vision(String cameraName, Transform3d robotToCam, AprilTagFieldLayout aprilTagFieldLayout) {
        camera = new PhotonCamera(cameraName);
        photonEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);
        photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    /**
     * Returns whether the camera pipeline has targets. Will call getLatestResult.
     * @return Whether the camera pipeline has targets
     */
    public boolean hasTargets() {
        return camera.getLatestResult().hasTargets();
    }

    public PhotonTrackedTarget getBestTarget() {
        return camera.getLatestResult().getBestTarget();
    }

    public void setLastPose(Pose2d pose) {
        photonEstimator.setLastPose(pose);
    }

    /**
     * The latest estimated robot pose on the field from vision data. This may be empty. This should
     * only be called once per loop. REMEMBER TO CONVERT TO FPGA TIME IF USING CTRE SWERVE!!!!!
     *
     * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
     *     used for estimation.
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (var res : camera.getAllUnreadResults()) {
            visionEst = photonEstimator.update(res);
        }
        return visionEst;
    }
}