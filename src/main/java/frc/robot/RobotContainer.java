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
import frc.robot.utils.ExtendedController;
import frc.robot.utils.ReefSelector;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.2).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final SendableChooser<Command> autoChooser;

    // Defining joysticks
    private final ExtendedController joystick = new ExtendedController(0);
    private final ExtendedController operator = new ExtendedController(1);

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

    private Function<ElevatorLocation, Command> prepareCoralToLevel = (location) -> {
        return Commands.parallel( 
            elevator.set(location),
            arm.set(ArmLocation.TOP),
            pivot.set(PivotLocation.OUTTAKE)
        );
    };

    private Command spitCoral = Commands.sequence(
        Commands.deadline(
            Commands.waitSeconds(1),
            arm.set(ArmLocation.SPIT),
            Commands.sequence(
                Commands.waitSeconds(0.5),
                intake.set(IntakeState.OUT)
            )
        ),
        Commands.parallel(
            arm.set(ArmLocation.INTAKE),
            Commands.sequence(
                Commands.waitSeconds(0.5),
                elevator.set(ElevatorLocation.BOTTOM),
                pivot.set(PivotLocation.INTAKE)
            )
        )
    );
    
    private Command spit = Commands.sequence(
        Commands.deadline(
            arm.set(ArmLocation.SPIT),
            Commands.sequence(
                Commands.waitSeconds(0.5),
                intake.set(IntakeState.OUT)
            )
        )
    );

    private Command toIntakePosition = Commands.parallel(
        arm.set(ArmLocation.INTAKE),
        Commands.sequence(
            Commands.waitSeconds(0.5),
            elevator.set(ElevatorLocation.BOTTOM),
            pivot.set(PivotLocation.INTAKE)
        )
    );


    @Deprecated
    private Function<ElevatorLocation, Command> putCoralToLevel = (location) -> {
        return Commands.sequence(
            Commands.parallel( 
                elevator.set(location),
                arm.set(ArmLocation.TOP),
                pivot.set(PivotLocation.OUTTAKE)
            ),
                Commands.waitSeconds(0.2),
                intake.set(IntakeState.OUT).repeatedly().withTimeout(1),
            Commands.parallel(
                pivot.set(PivotLocation.INTAKE),
                arm.set(ArmLocation.INTAKE),
                elevator.set(ElevatorLocation.BOTTOM)
            )
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
                    Commands.parallel(
                        //Commands.defer(() -> drivetrain.driveToPose(reefSelector.getSelectedPose()), Set.of(drivetrain)),
                        Commands.defer(() -> prepareCoralToLevel.apply(reefSelector.getElevatorLocation()), Set.of(elevator, arm, pivot, intake))
                    )
                ));
            }
        }

        NamedCommands.registerCommand("Spit", spit);

        NamedCommands.registerCommand("ReturnToIntakePosition", toIntakePosition);

        NamedCommands.registerCommand("GetCoral", intake.set(IntakeState.IN).repeatedly().withTimeout(0.5));

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
            Commands.parallel(
                Commands.defer(() -> drivetrain.driveToPose(reefSelector.getSelectedPose()), Set.of(drivetrain)),
                Commands.defer(() -> prepareCoralToLevel.apply(reefSelector.getElevatorLocation()), Set.of(elevator, arm, pivot))
            )
        );

        joystick.a().whileTrue(
            Commands.parallel( 
                elevator.set(ElevatorLocation.DOWNINTAKE),
                arm.set(ArmLocation.GROUND),
                intake.set(IntakeState.IN).repeatedly(),
                pivot.set(PivotLocation.INTAKE)
            )
        );
        
        joystick.a().toggleOnFalse(prepareCoralToLevel.apply(ElevatorLocation.BOTTOM));

        joystick.x().onTrue(Commands.parallel(
            elevator.set(ElevatorLocation.BOTTOM),
            pivot.set(PivotLocation.INTAKE),
            arm.set(ArmLocation.INTAKE),
            intake.set(IntakeState.IN).repeatedly().withTimeout(3)
        ));

        // joystick.a().onTrue(putCoralToLevel.apply(ElevatorLocation.BOTTOM));
        // joystick.b().onTrue(putCoralToLevel.apply(ElevatorLocation.MID));
        // joystick.povUp().onTrue(putCoralToLevel.apply(ElevatorLocation.TOP));
        joystick.b().onTrue(spitCoral);

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

        operator.y().onTrue(Commands.sequence(
            pivot.set(PivotLocation.INTAKE),
            arm.set(ArmLocation.DEFAULT),
            elevator.set(ElevatorLocation.BOTTOM)
        ));  

        operator.rightTrigger().whileTrue(Commands.parallel(
            algaeIntake.set(AlgaeIntakeState.IN).repeatedly(),
            algaePivot.set(AlgaePivotLocation.INTAKE)
        ));
        operator.leftTrigger().whileTrue(Commands.parallel(
            algaeIntake.set(AlgaeIntakeState.OUT).repeatedly(),
            algaePivot.set(AlgaePivotLocation.DEFAULT)
        ));

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

