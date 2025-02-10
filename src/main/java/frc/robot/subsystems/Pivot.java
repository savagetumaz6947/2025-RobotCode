package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pivot extends SubsystemBase {
    private TalonFX motor = new TalonFX(4, "rio");
    final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(3.75).withSlot(0);

    private static FlywheelSim sim;
    public static MechanismLigament2d mech2d;

    public enum PivotLocation {
        INTAKE, OUTTAKE, UNDEFINED
    }

    private Map<PivotLocation, Double> locationsMap = new HashMap<>();
    private PivotLocation state = PivotLocation.INTAKE;

    public Pivot() {
        Slot0Configs pivotSlot0Configs = new Slot0Configs();
        pivotSlot0Configs.kP = 0.6;
        pivotSlot0Configs.kI = 0.1;
        pivotSlot0Configs.kD = 0.2;
        pivotSlot0Configs.kV = 0.15;
        pivotSlot0Configs.kA = 0.025;

        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicAcceleration = 400;
        motionMagicConfigs.MotionMagicCruiseVelocity = 800;
        motionMagicConfigs.MotionMagicJerk = 800; 

        motor.getConfigurator().apply(pivotSlot0Configs);
        motor.getConfigurator().apply(motionMagicConfigs);
        motor.setNeutralMode(NeutralModeValue.Brake);
        motor.setPosition(0);

        locationsMap.put(PivotLocation.INTAKE, -0.2);
        locationsMap.put(PivotLocation.OUTTAKE, 12.0);

        this.setDefaultCommand(this.set(() -> 0).repeatedly());
    }

    public Command set(DoubleSupplier voltage) {
        return this.run(() -> {
            motor.setVoltage(voltage.getAsDouble());
            if (voltage.getAsDouble() != 0) state = PivotLocation.UNDEFINED;
        });
    }

    public Command set(PivotLocation location) {
        return this.run(() -> {
            double targetPosition = locationsMap.get(location);
            motor.setControl(motionMagicRequest.withPosition(targetPosition));
            state = location;
        }).until(() -> MathUtil.isNear(locationsMap.get(location), motor.getPosition().getValueAsDouble(), 1));
    }

    public PivotLocation getState() {
        return state;
    }

    public Command eStop() {
        return this.runOnce(() -> {
            motor.setVoltage(0);
            state = PivotLocation.UNDEFINED;
        });
    }
    @Override
    public void simulationPeriodic() {
        motor.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());
        sim.setInputVoltage(motor.getSimState().getMotorVoltage());
        sim.update(0.02);

        motor.getSimState().addRotorPosition(Units.radiansToRotations(sim.getAngularVelocityRadPerSec()) * TimedRobot.kDefaultPeriod * 15);
        motor.getSimState().setRotorVelocity(RPM.of(sim.getAngularVelocityRPM()));

        mech2d.setAngle(Rotation2d.fromRotations(motor.getPosition().getValueAsDouble() / 15));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pivot/Motor/EncoderPos", motor.getPosition().getValueAsDouble());
    }
}
