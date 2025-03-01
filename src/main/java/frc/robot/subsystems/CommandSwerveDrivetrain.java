package frc.robot.subsystems;

import java.util.function.Supplier;

import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.command.DriveToPose;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms

    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private RobotConfig config;

    private ExtendedPhotonCamera visionDown = new ExtendedPhotonCamera(Constants.VisionDownCam.CAMERA_NAME, Constants.VisionDownCam.ROBOT_TO_CAM, Constants.VisionDownCam.APRIL_TAG_FIELD_LAYOUT);
    private ExtendedPhotonCamera visionUp = new ExtendedPhotonCamera(Constants.VisionUpCam.CAMERA_NAME, Constants.VisionUpCam.ROBOT_TO_CAM, Constants.VisionUpCam.APRIL_TAG_FIELD_LAYOUT);

    private VisionSystemSim visionSim;
    private PhotonCameraSim cameraDownSim;
    private PhotonCameraSim cameraUpSim;

    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    public static Field2d field2d = new Field2d();

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
     * @param modules               Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            configureSimulation();
        }
        configurePPLib();
        SmartDashboard.putData("Field", field2d);
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            configureSimulation();
        }
        configurePPLib();
        SmartDashboard.putData("Field", field2d);
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry calculation
     *                                  in the form [x, y, theta], with units in meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision calculation
     *                                  in the form [x, y, theta], with units in meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) {
            configureSimulation();
        }
        configurePPLib();
        SmartDashboard.putData("Field", field2d);
    }

    private void configurePPLib() {
        try{
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
        }
        AutoBuilder.configure(
            () -> super.getState().Pose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            () -> super.getState().Speeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> setControl(
                m_pathApplyRobotSpeeds.withSpeeds(speeds)
                    .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                    .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
            ), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                new PIDConstants(7.5, 0.0, 0.05), // Translation PID constants500
                new PIDConstants(3, 0.0, 0.05) // Rotation PID constants500
    
            ),
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
        );
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return this.run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Returns a command that drives this swerve drivetrain to a given target pose. This function will attempt to create a linear path from the current pose.
     * Use Commands.defer when constructing this command: Commands.defer(() -> drivetrain.driveToPose(reefSelector.getSelectedPose()), Set.of(drivetrain)).
     * 
     * @param targetPose The field-relative target pose
     * @return Command to run from PPLib's AutoBuilder
     */
    public Command driveToPose(Pose2d targetPose) {
        // Example usage
        // joystick.rightBumper().whileTrue(
        //     Commands.sequence(
        //         Commands.defer(() -> drivetrain.driveToPose(reefSelector.getSelectedPose()), Set.of(drivetrain)),
        //         Commands.defer(() -> putCoralToLevel.apply(reefSelector.getElevatorLocation()), Set.of(elevator, arm, pivot))
        //     )
        // );

        if (Constants.Drivetrain.USE_PPLIB_FOR_AUTOALIGN) {
            // USE PPLib
            Translation2d currLocation = this.getState().Pose.getTranslation();
            Rotation2d currToTargetAngle = Rotation2d.fromRadians(Math.atan2(targetPose.getY() - currLocation.getY(), targetPose.getX() - currLocation.getX()));//算法

            PathPlannerPath path = new PathPlannerPath(
                PathPlannerPath.waypointsFromPoses(
                    new Pose2d(currLocation, currToTargetAngle),
                    new Pose2d(targetPose.getTranslation(), currToTargetAngle)
                ),
                Constants.Drivetrain.DRIVE_TO_POSE_CONSTRAINTS,
                null,
                new GoalEndState(0.0, targetPose.getRotation()));
            path.preventFlipping = true;
            
            try {
                path.generateTrajectory(new ChassisSpeeds(), currLocation.getAngle(), config);
            } catch (Exception e) {
                return this.runOnce(() -> {});
            }

            return AutoBuilder.followPath(path);
        } else {
            return new DriveToPose(this, targetPose);
        }
    }

    public static void setSelectedReef(Pose2d pose) {
        field2d.getObject("selectedReef").setPose(pose);
    }

    private void configureSimulation() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);

        // Setup PhotonVision Simulation
        visionSim = new VisionSystemSim("main");
        visionSim.addAprilTags(Constants.VisionDownCam.APRIL_TAG_FIELD_LAYOUT);

        SimCameraProperties cameraDownProperties = new SimCameraProperties();
        cameraDownProperties.setCalibration(Constants.VisionDownCam.Simulated.WIDTH, Constants.VisionDownCam.Simulated.HEIGHT, Constants.VisionDownCam.Simulated.FOV);
        cameraDownProperties.setCalibError(0.01, 0.08);
        cameraDownProperties.setFPS(Constants.VisionDownCam.Simulated.FPS);
        cameraDownProperties.setAvgLatencyMs(35);
        cameraDownProperties.setLatencyStdDevMs(5);
        cameraDownSim = new PhotonCameraSim(visionDown.getCamera(), cameraDownProperties);

        cameraDownSim.enableRawStream(true);
        cameraDownSim.enableProcessedStream(true);
        cameraDownSim.enableDrawWireframe(true);

        visionSim.addCamera(cameraDownSim, Constants.VisionDownCam.ROBOT_TO_CAM);

        SimCameraProperties cameraUpProperties = new SimCameraProperties();
        cameraUpProperties.setCalibration(Constants.VisionUpCam.Simulated.WIDTH, Constants.VisionUpCam.Simulated.HEIGHT, Constants.VisionUpCam.Simulated.FOV);
        cameraUpProperties.setCalibError(0.01, 0.08);
        cameraUpProperties.setFPS(Constants.VisionUpCam.Simulated.FPS);
        cameraUpProperties.setAvgLatencyMs(35);
        cameraUpProperties.setLatencyStdDevMs(5);
        cameraUpSim = new PhotonCameraSim(visionUp.getCamera(), cameraUpProperties);

        cameraUpSim.enableRawStream(true);
        cameraUpSim.enableProcessedStream(true);
        cameraUpSim.enableDrawWireframe(true);

        visionSim.addCamera(cameraUpSim, Constants.VisionUpCam.ROBOT_TO_CAM);
    }

    @Override
    public void simulationPeriodic() {
        visionSim.update(this.getState().Pose);
        updateSimState(0.02, RobotController.getBatteryVoltage());
    }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }

        var visionDownEst = visionDown.getEstimatedGlobalPose();
        var visionUpEst = visionUp.getEstimatedGlobalPose();
            visionDownEst.ifPresent(
                est -> {
                    // Hours wasted because CTRE decided to use FPGA Time: 5
                    this.addVisionMeasurement(est.estimatedPose.toPose2d(), Utils.fpgaToCurrentTime(est.timestampSeconds));
            });
        if (RobotState.isDisabled()) {
            visionUpEst.ifPresent(
                est -> {
                    // Hours wasted because CTRE decided to use FPGA Time: 5
                    this.addVisionMeasurement(est.estimatedPose.toPose2d(), Utils.fpgaToCurrentTime(est.timestampSeconds));
            });
        }

        field2d.setRobotPose(this.getState().Pose);
    }
}