package frc.robot.command;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DriveToPose extends Command {
    CommandSwerveDrivetrain swerve;
    Pose2d targetPose;

    private static double kDt = 0.02;

    private final SwerveRequest.FieldCentricFacingAngle centricControl = new SwerveRequest.FieldCentricFacingAngle()
            .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
            .withHeadingPID(Constants.Drivetrain.DriveToPose.ROTATION_kP, Constants.Drivetrain.DriveToPose.ROTATION_kI,
                    Constants.Drivetrain.DriveToPose.ROTATION_kD);
    private final SwerveRequest.SwerveDriveBrake brakeControl = new SwerveRequest.SwerveDriveBrake();

    private PIDController xPidController = new PIDController(Constants.Drivetrain.DriveToPose.TRANSLATION_kP,
            Constants.Drivetrain.DriveToPose.TRANSLATION_kI, Constants.Drivetrain.DriveToPose.TRANSLATION_kD);
    private PIDController yPidController = new PIDController(Constants.Drivetrain.DriveToPose.TRANSLATION_kP,
            Constants.Drivetrain.DriveToPose.TRANSLATION_kI, Constants.Drivetrain.DriveToPose.TRANSLATION_kD);

    private TrapezoidProfile xTrapezoidProfile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(Constants.Drivetrain.DriveToPose.CONSTRAINTS.maxVelocityMPS(),
                    Constants.Drivetrain.DriveToPose.CONSTRAINTS.maxAccelerationMPSSq()));
    private TrapezoidProfile yTrapezoidProfile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(Constants.Drivetrain.DriveToPose.CONSTRAINTS.maxVelocityMPS(),
                    Constants.Drivetrain.DriveToPose.CONSTRAINTS.maxAccelerationMPSSq()));

    TrapezoidProfile.State xSetpoint;
    TrapezoidProfile.State ySetpoint;

    public DriveToPose(CommandSwerveDrivetrain swerve, Pose2d targetPose) {
        this.swerve = swerve;
        this.targetPose = targetPose;

        // Configure PID controllers for driveToPose()
        xPidController.setTolerance(Constants.Drivetrain.DriveToPose.TRANSLATION_TOLERANCE);
        yPidController.setTolerance(Constants.Drivetrain.DriveToPose.TRANSLATION_TOLERANCE);
        centricControl.HeadingController.setTolerance(Constants.Drivetrain.DriveToPose.ROTATION_TOLERANCE);

        xSetpoint = new TrapezoidProfile.State(swerve.getState().Pose.getX(),
                swerve.getState().Speeds.vxMetersPerSecond);
        ySetpoint = new TrapezoidProfile.State(swerve.getState().Pose.getY(),
                swerve.getState().Speeds.vyMetersPerSecond);
    }

    @Override
    public void execute() {
        xSetpoint = xTrapezoidProfile.calculate(kDt, xSetpoint, new TrapezoidProfile.State(targetPose.getX(), 0));
        ySetpoint = yTrapezoidProfile.calculate(kDt, ySetpoint, new TrapezoidProfile.State(targetPose.getY(), 0));

        xPidController.setSetpoint(xSetpoint.position);
        yPidController.setSetpoint(ySetpoint.position);

        swerve.setControl(centricControl
                .withVelocityX(xPidController.calculate(swerve.getState().Pose.getX()))
                .withVelocityY(yPidController.calculate(swerve.getState().Pose.getY()))
                .withTargetDirection(targetPose.getRotation()));
    }

    @Override
    public boolean isFinished() {
        return centricControl.HeadingController.atSetpoint() &&
                MathUtil.isNear(targetPose.getX(), swerve.getState().Pose.getX(),
                        Constants.Drivetrain.DriveToPose.TRANSLATION_TOLERANCE)
                &&
                MathUtil.isNear(targetPose.getY(), swerve.getState().Pose.getY(),
                        Constants.Drivetrain.DriveToPose.TRANSLATION_TOLERANCE);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setControl(brakeControl);
    }
}
