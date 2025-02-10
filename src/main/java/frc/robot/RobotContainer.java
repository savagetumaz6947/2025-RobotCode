// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmLocation;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorLocation;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.algaeIntake;
import frc.robot.subsystems.algaePivot;
import frc.robot.subsystems.Pivot.PivotLocation;
import frc.robot.subsystems.algaeIntake.algaeIntakeState;
import frc.robot.subsystems.algaePivot.algaePivotLocation;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.2).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final SendableChooser<Command> autoChooser;

    // Defining joysticks
    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    // Defining subsystems
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final Intake intake = new Intake();
    private final Pivot pivot = new Pivot();
    private final Arm arm = new Arm();
    private final Elevator elevator = new Elevator();
    private final algaeIntake algaeIntake = new algaeIntake();
    private final algaePivot algaePivot = new algaePivot();

    private Function<ElevatorLocation, Command> putCoralToLevel = (location) -> {
        return new SequentialCommandGroup(
            new ParallelCommandGroup( 
                elevator.set(location),
                arm.set(ArmLocation.OUTTAKE),
                pivot.set(PivotLocation.OUTTAKE)
            ),
                new WaitCommand(0.2),
                intake.set(IntakeState.OUT).repeatedly().withTimeout(0.5),
                arm.set(ArmLocation.OUT),
            new ParallelCommandGroup(
                pivot.set(PivotLocation.INTAKE),
                arm.set(ArmLocation.INTAKE),
                elevator.set(ElevatorLocation.BOTTOM)
            )
        );
    };

    public RobotContainer() {
        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData("Auto Chooser", autoChooser);

        configureBindings();
    }
    private DoubleSupplier speedSupplier = () -> MaxSpeed * 0.2;

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * speedSupplier.getAsDouble()) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * speedSupplier.getAsDouble()) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        joystick.rightTrigger().onTrue(new InstantCommand(() -> {
            speedSupplier = () -> MaxSpeed * 0.35;
        }));
        joystick.rightTrigger().onFalse(new InstantCommand(() -> {
            speedSupplier = () -> MaxSpeed * 0.2; // default speed is 20% theoretical max speed
        }));
        
        joystick.leftTrigger().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.rightBumper().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        joystick.povDown().onTrue(new SequentialCommandGroup(
            new ParallelCommandGroup( 
                elevator.set(ElevatorLocation.BOTTOM),
                arm.set(ArmLocation.GROUND),
                pivot.set(PivotLocation.OUTTAKE)
            ),
                pivot.set(PivotLocation.INTAKE),
                pivot.set(PivotLocation.OUTTAKE),
                new WaitCommand(0.2),
                intake.set(IntakeState.IN).repeatedly().withTimeout(1),
                arm.set(ArmLocation.OUTTAKE),
            new ParallelCommandGroup(
                arm.set(ArmLocation.DEFAULT),
                elevator.set(ElevatorLocation.BOTTOM)
            )
        ));

        joystick.x().onTrue(new SequentialCommandGroup(
            elevator.set(ElevatorLocation.BOTTOM),
            pivot.set(PivotLocation.INTAKE),
            arm.set(ArmLocation.INTAKE),
            intake.set(IntakeState.IN).repeatedly().withTimeout(3)
            )
        );
            
        joystick.b().onTrue(putCoralToLevel.apply(ElevatorLocation.MID));
        joystick.povUp().onTrue(putCoralToLevel.apply(ElevatorLocation.TOP));
        joystick.y().onTrue(new SequentialCommandGroup(
            pivot.set(PivotLocation.INTAKE),
            arm.set(ArmLocation.DEFAULT),
            elevator.set(ElevatorLocation.BOTTOM)
        ));            

        joystick.leftBumper().or(operator.leftBumper()).onTrue(new ParallelCommandGroup(
            arm.eStop(),
            pivot.eStop(),
            elevator.eStop(),
            new InstantCommand(() -> {}, drivetrain)
        ));
//-------------------------------------------------------------------------------------------------------------------  
        
        joystick.a().onTrue(putCoralToLevel.apply(ElevatorLocation.BOTTOM));
//-------------------------------------------------------------------------------------------------------------------

        //operator.y().whileTrue(elevator.set(operator::getLeftX).repeatedly());
        //operator.a().whileTrue(arm.set(operator::getLeftX).repeatedly());

        // arm.setDefaultCommand(arm.set(() -> operator.getLeftX()));
        // operator.x().onTrue(arm.set(ArmLocation.OUTTAKE));
        // operator.b().onTrue(arm.set(ArmLocation.INTAKE));
        //operator.a().onTrue(elevator.set(ElevatorLocation.MID));
        operator.x().onTrue(algaeIntake.set(algaeIntakeState.OUT).repeatedly().withTimeout(0.6));
        operator.b().onTrue(algaeIntake.set(algaeIntakeState.IN).repeatedly().withTimeout(0.6));
        operator.y().onTrue(algaePivot.set(algaePivotLocation.INTAKE));
        operator.a().onTrue(algaePivot.set(algaePivotLocation.OUTTAKE));
        // operator.rightBumper().onTrue(elevator.set(ElevatorLocation.MID));
        // operator.leftBumper().onTrue(elevator.set(ElevatorLocation.TOP));
        // operator.rightTrigger().onTrue(elevator.set(ElevatorLocation.BOTTOM));
        
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}

