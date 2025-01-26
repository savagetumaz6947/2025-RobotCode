package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;

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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    private TalonFX motor = new TalonFX(3, "rio");
    private final PositionVoltage request = new PositionVoltage(0.9).withSlot(0);

    private static SingleJointedArmSim sim;
    private static MechanismLigament2d mech2d;

    public enum ArmLocation{
        INTAKE, OUTTAKE, UNDEFINED
    }

    private Map<ArmLocation, Double> locationsMap = new HashMap<>();

    private ArmLocation state = ArmLocation.INTAKE;

    public Arm () {
        Slot0Configs armSlot0Configs = new Slot0Configs();
        armSlot0Configs.kP = 0.6;
        armSlot0Configs.kI = 0.15;
        armSlot0Configs.kD = 0.085;

        motor.getConfigurator().apply(armSlot0Configs);
        motor.setPosition(0);
        motor.setNeutralMode(NeutralModeValue.Brake);

        locationsMap.put(ArmLocation.INTAKE, degreeToEncoder(-32));
        locationsMap.put(ArmLocation.OUTTAKE, degreeToEncoder(25));

        this.setDefaultCommand(this.set(() -> 0.0).repeatedly());
    }

    public Command set(DoubleSupplier volt){
        return this.runOnce(() -> {
            motor.setVoltage(volt.getAsDouble()*1 + getFeedForward());
            if (volt.getAsDouble() != 0) state = ArmLocation.UNDEFINED;
        });
    }

    public Command set(ArmLocation location){
        return this.run(() -> {
            motor.setControl(request.withPosition(locationsMap.get(location)).withFeedForward(getFeedForward()));
            state = location;
        }).until(() -> MathUtil.isNear(locationsMap.get(location), motor.getPosition().getValueAsDouble(), 0.05));
    }

    public double getFeedForward() {
        return Math.cos(Math.toRadians(90 - getAbsoluteDegrees())) * -0.42;
    }

    private double getAbsoluteDegrees() {
        return motor.getPosition().getValueAsDouble() * 11.09 - 32;
    }

    private double degreeToEncoder(double degree) {
        return (degree + 32)/11.09;
    }

    public ArmLocation getState() {
        return state;
    }

    public Command eStop() {
        return this.runOnce(() -> {
            motor.setVoltage(0);
            state = ArmLocation.UNDEFINED;
        });
    }

    public void configureSimulation() {
        // The origin is the x-axis on a mathematical cartesian plane -->
        sim = new SingleJointedArmSim(DCMotor.getKrakenX60(1), 32.46, .22, 0.4, Units.degreesToRadians(-90), Units.degreesToRadians(270), true, Units.degreesToRadians(90-32));
        mech2d = Elevator.getMech2d().append(new MechanismLigament2d("Arm", 0.4, 90-32, 10, new Color8Bit(Color.kDarkGreen)));
    }

    @Override
    public void simulationPeriodic() {
        motor.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());
        sim.setInputVoltage(motor.getSimState().getMotorVoltage());
        sim.update(0.02);

        mech2d.setAngle(Rotation2d.fromRadians(sim.getAngleRads() - Math.PI/2));

        motor.getSimState().setRawRotorPosition(degreeToEncoder(Units.radiansToDegrees(sim.getAngleRads()) - 90));
        motor.getSimState().setRotorVelocity(RotationsPerSecond.of(degreeToEncoder(Units.radiansToDegrees(sim.getVelocityRadPerSec()))));

        SmartDashboard.putNumber("Arm/Sim/SimDeg", Units.radiansToDegrees(sim.getAngleRads()));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm/Motor/EncoderPos", motor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Arm/Motor/AbsoluteDegree", getAbsoluteDegrees());
        SmartDashboard.putNumber("Arm/Motor/MotorVoltage", motor.getMotorVoltage().getValueAsDouble());
    }
}
