package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Pivot extends SubsystemBase {
    private TalonFX motor = new TalonFX(4, "rio");
    final PositionVoltage request = new PositionVoltage(3.75).withSlot(0);

    private static FlywheelSim sim;
    public static MechanismLigament2d mech2d;

    public enum PivotLocation {
        INTAKE, OUTTAKE, UNDEFINED
    }

    private Map<PivotLocation, Double> locationsMap = new HashMap<>();

    private PivotLocation state = PivotLocation.INTAKE;

    public Pivot () {
        Slot0Configs pivotSlot0Configs = new Slot0Configs();
        pivotSlot0Configs.kP = 1.05;
        pivotSlot0Configs.kI = 0.35;
        pivotSlot0Configs.kD = 0.085;

        motor.getConfigurator().apply(pivotSlot0Configs);
        motor.setNeutralMode(NeutralModeValue.Brake);
        motor.setPosition(0);

        locationsMap.put(PivotLocation.INTAKE, 0.0);
        locationsMap.put(PivotLocation.OUTTAKE, 3.75);

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
            motor.setControl(request.withPosition(locationsMap.get(location)));
            state = location;
        }).until(() -> {
            return MathUtil.isNear(locationsMap.get(location), motor.getPosition().getValueAsDouble(), 0.05);
        });
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

    public void configureSimulation() {
        sim = new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(1), 0.05, 15), DCMotor.getKrakenX60(1));
        mech2d = RobotContainer.smallMech2dRoot.append(new MechanismLigament2d("Pivot", 0.5, 0, 10, new Color8Bit(Color.kAqua)));
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
