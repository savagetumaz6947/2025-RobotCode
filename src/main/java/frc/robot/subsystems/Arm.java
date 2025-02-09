package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;

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
    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0.9).withSlot(0);

    private static SingleJointedArmSim sim;
    private static MechanismLigament2d mech2d;

    public enum ArmLocation {
        INTAKE, OUTTAKE, OUT, UNDEFINED, DEFAULT,
    }

    private Map<ArmLocation, Double> locationsMap = new HashMap<>();
    private ArmLocation state = ArmLocation.INTAKE;

    public Arm() {
        
        Slot0Configs armSlot0Configs = new Slot0Configs();
        armSlot0Configs.kP = 0.5;
        armSlot0Configs.kI = 0;
        armSlot0Configs.kD = 0.2;
        armSlot0Configs.kV = 0.25;
        armSlot0Configs.kA = 0.1;

        
        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicAcceleration = 25; 
        motionMagicConfigs.MotionMagicCruiseVelocity = 100; 
        motionMagicConfigs.MotionMagicJerk = 200;

        
        motor.getConfigurator().apply(armSlot0Configs);
        motor.getConfigurator().apply(motionMagicConfigs);

        motor.setPosition(0);
        motor.setNeutralMode(NeutralModeValue.Brake);

        locationsMap.put(ArmLocation.INTAKE, -13.0);
        locationsMap.put(ArmLocation.OUTTAKE, 24.0);
        locationsMap.put(ArmLocation.OUT, 10.0);
        locationsMap.put(ArmLocation.DEFAULT, 0.0);

        this.setDefaultCommand(this.set(() -> 0.0).repeatedly());
    }

    public Command set(DoubleSupplier volt) {
        return this.runOnce(() -> {
            motor.setVoltage(volt.getAsDouble() * 1);
            if (volt.getAsDouble() != 0) state = ArmLocation.UNDEFINED;
        });
    }

    public Command set(ArmLocation location) {
        return this.run(() -> {
            motor.setControl(motionMagicRequest.withPosition(locationsMap.get(location)));
            state = location;
        }).until(() -> MathUtil.isNear(locationsMap.get(location), motor.getPosition().getValueAsDouble(), 0.5));
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
        SmartDashboard.putNumber("Arm/Motor/MotorVoltage", motor.getMotorVoltage().getValueAsDouble());
    }
}
