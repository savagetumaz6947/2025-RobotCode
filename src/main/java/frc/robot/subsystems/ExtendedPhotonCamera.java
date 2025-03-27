package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Distance;

public class ExtendedPhotonCamera {
    private PhotonCamera camera;
    private PhotonPoseEstimator photonEstimator;
    private Distance maxDistance;
    private double maxAmbiguity;

    /**
     * Constructs a ExtendedPhotonCamera object with no transformation (primarily for use with Object Detection)
     * @param cameraName The name of the PhotonCamera
     */
    public ExtendedPhotonCamera(String cameraName) {
        this(cameraName, new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0)));
    }

    /**
     * Constructs a ExtendedPhotonCamera object with the kDefaultField field layout
     * @param cameraName The name of the PhotonCamera
     * @param robotToCam The location of the camera relative to the robot center
     */
    public ExtendedPhotonCamera(String cameraName, Transform3d robotToCam) {
        this(cameraName, robotToCam, AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField), Meters.of(1), 0.15);
    }

    /**
     * Constructs a ExtendedPhotonCamera object with custom AprilTagFieldLayout
     * @param cameraName The name of the PhotonCamera
     * @param robotToCam The location of the camera relative to the robot center
     * @param aprilTagFieldLayout The custom AprilTagFieldLayout
     * @param maxDistance The maximum accepted distance from the camera to the "best" AprilTag.
     * @param maxAmbiguity The maximum accepted ambiguity from the camera to the "best" AprilTag.
     */
    public ExtendedPhotonCamera(String cameraName, Transform3d robotToCam, AprilTagFieldLayout aprilTagFieldLayout,
        Distance maxDistance, double maxAmbiguity) {
        camera = new PhotonCamera(cameraName);
        photonEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);
        photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        this.maxDistance = maxDistance;
        this.maxAmbiguity = maxAmbiguity;
    }

    public PhotonCamera getCamera() {
        return camera;
    }

    /**
     * The latest estimated robot pose on the field from vision data. This may be empty. This should
     * only be called once per loop. REMEMBER TO CONVERT TO FPGA TIME IF USING CTRE SWERVE!!!!!
     *
     * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
     *     used for estimation.
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(boolean overrideCheck) {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();

        for (var res : camera.getAllUnreadResults()) {
            Optional<EstimatedRobotPose> photonPose = photonEstimator.update(res);

            if (photonPose.isPresent()) {
                double tag0Dist = res.getBestTarget().bestCameraToTarget.getTranslation().getNorm();
                double poseAmbaguitiy = res.getBestTarget().getPoseAmbiguity();

                if (overrideCheck || (tag0Dist < maxDistance.in(Meters) && poseAmbaguitiy < maxAmbiguity)) {
                    visionEst = photonPose;
                }
            }
        }

        return visionEst;
    }
}