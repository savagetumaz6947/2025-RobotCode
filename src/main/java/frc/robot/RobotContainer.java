// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Function;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Arm.ArmLocation;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorLocation;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LedStrip;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.AlgaeIntake.AlgaeIntakeState;
import frc.robot.subsystems.AlgaePivot.AlgaePivotLocation;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.AlgaePivot;
import frc.robot.subsystems.Pivot.PivotLocation;
import frc.robot.utils.ReefSelector;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.2).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final SendableChooser<Command> autoChooser;

    // Defining joysticks
    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    // Defining simulation constants
    // BIG holds elevator and arm
    public static Mechanism2d bigMech2d = new Mechanism2d(2, 2);
    public static MechanismRoot2d bigMech2dRoot = bigMech2d.getRoot("Structure Root", 1, 0.2);
    // SMALL holds pivot and intake
    public static Mechanism2d smallMech2d = new Mechanism2d(2, 2);
    public static MechanismRoot2d smallMech2dRoot = smallMech2d.getRoot("Structure Root", 1, 0.2);

    // Defining subsystems
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final Elevator elevator = new Elevator();
    private final Arm arm = new Arm();
    private final Pivot pivot = new Pivot();
    private final Intake intake = new Intake();
    private final AlgaeIntake algaeIntake = new AlgaeIntake();
    private final AlgaePivot algaePivot = new AlgaePivot();
    private final LedStrip ledStrip = new LedStrip(() -> elevator.getHeight());
    private final Climber climber = new Climber();

    private final ReefSelector reefSelector = new ReefSelector();

    private Function<ElevatorLocation, Command> putCoralToLevel = (location) -> {
        return Commands.sequence(
            Commands.parallel( 
                elevator.set(location),
                arm.set(ArmLocation.TOP),
                pivot.set(PivotLocation.OUTTAKE)
            ),
                Commands.waitSeconds(0.2),
                intake.set(IntakeState.OUT).repeatedly().withTimeout(1),
                arm.set(ArmLocation.OUT),
            Commands.parallel(
                pivot.set(PivotLocation.INTAKE),
                arm.set(ArmLocation.INTAKE),
                elevator.set(ElevatorLocation.BOTTOM)
            )
        );
    };

    private Function<ElevatorLocation, Command> riseToLevel = (location) -> {
        return Commands.parallel( 
                elevator.set(location),
                arm.set(ArmLocation.TOP),
                pivot.set(PivotLocation.OUTTAKE)
        );
    };

    public RobotContainer() {
        // Build an auto chooser. This will use Commands.none() as the default option.
        NamedCommands.registerCommand("PutArmSafe", arm.set(ArmLocation.INTAKE));
        for (char r = 'A'; r <= 'L'; r++) {
            for (int i = 2; i <= 4; i++) {
                final char fr = r;
                final int fi = i;
                NamedCommands.registerCommand("Put" + fr + i, Commands.sequence(
                    Commands.runOnce(() -> reefSelector.setReef(fr, fi)),
                    Commands.defer(() -> drivetrain.driveToPose(reefSelector.getSelectedPose()), Set.of(drivetrain)),
                    Commands.defer(() -> putCoralToLevel.apply(reefSelector.getElevatorLocation()), Set.of(elevator, arm, pivot, intake))
                ));
            }
        }

        NamedCommands.registerCommand("GetCoral", intake.set(IntakeState.IN).repeatedly().withTimeout(3));

        autoChooser = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData("Auto Chooser", autoChooser);

        configureBindings();


        if (Robot.isSimulation()) {
            SmartDashboard.putData("Sim/BigMechanism", bigMech2d);
            SmartDashboard.putData("Sim/SmallMechanism", smallMech2d);
        }
    }
    private DoubleSupplier speedSupplier = () -> MaxSpeed * 0.25;

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
        climber.setDefaultCommand(climber.setVoltageCommand(() -> operator.getRightY()));

        joystick.rightTrigger().onTrue(Commands.runOnce(() -> {
            speedSupplier = () -> MaxSpeed * 0.4;
        }));
        joystick.rightTrigger().onFalse(Commands.runOnce(() -> {
            speedSupplier = () -> MaxSpeed * 0.2; // default speed is 20% theoretical max speed
        }));
        
        joystick.rightBumper().onTrue(
            Commands.sequence(
                Commands.defer(() -> drivetrain.driveToPose(reefSelector.getSelectedPose()), Set.of(drivetrain))
                //Commands.defer(() -> putCoralToLevel.apply(reefSelector.getElevatorLocation()), Set.of(elevator, arm, pivot, intake))
            )
        );

        joystick.leftTrigger().whileTrue(drivetrain.applyRequest(() -> brake));

        joystick.povDown().onTrue(Commands.sequence(
            Commands.parallel( 
                elevator.set(ElevatorLocation.BOTTOM),
                arm.set(ArmLocation.GROUND)
            ),
                intake.set(IntakeState.IN).repeatedly().withTimeout(5)
        ));

        joystick.x().onTrue(Commands.sequence(
            elevator.set(ElevatorLocation.BOTTOM),
            pivot.set(PivotLocation.INTAKE),
            arm.set(ArmLocation.INTAKE),
            intake.set(IntakeState.IN).repeatedly().withTimeout(3)
            )
        );

        joystick.a().onTrue(putCoralToLevel.apply(ElevatorLocation.BOTTOM));
        joystick.b().onTrue(putCoralToLevel.apply(ElevatorLocation.MID));
        joystick.povUp().onTrue(putCoralToLevel.apply(ElevatorLocation.TOP));

        joystick.y().onTrue(Commands.sequence(
            pivot.set(PivotLocation.INTAKE),
            arm.set(ArmLocation.DEFAULT),
            elevator.set(ElevatorLocation.BOTTOM)
        ));            

        joystick.leftBumper().or(operator.leftBumper()).onTrue(Commands.parallel(
            arm.eStop(),
            pivot.eStop(),
            elevator.eStop(),
            intake.eStop(),
            Commands.runOnce(() -> {}, drivetrain)
        ));

        operator.rightTrigger().onTrue(algaeIntake.set(AlgaeIntakeState.IN).repeatedly().withTimeout(0.6));
        operator.leftTrigger().onTrue(algaeIntake.set(AlgaeIntakeState.OUT).repeatedly().withTimeout(0.6));
        operator.x().onTrue(algaePivot.set(AlgaePivotLocation.INTAKE));
        operator.a().onTrue(algaePivot.set(AlgaePivotLocation.DEFAULT));

        operator.b().onTrue(intake.set(IntakeState.OUT).repeatedly().withTimeout(0.6));

        operator.rightBumper().onTrue(Commands.sequence(
            elevator.set(ElevatorLocation.ALGAE),
            arm.set(ArmLocation.ALGAE),

            intake.set(IntakeState.OUT)
        ));

        operator.povUp().onTrue(Commands.runOnce(() -> reefSelector.goUp()));
        operator.povDown().onTrue(Commands.runOnce(() -> reefSelector.goDown()));
        operator.povLeft().onTrue(Commands.runOnce(() -> reefSelector.goLeft()));
        operator.povRight().onTrue(Commands.runOnce(() -> reefSelector.goRight()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}

