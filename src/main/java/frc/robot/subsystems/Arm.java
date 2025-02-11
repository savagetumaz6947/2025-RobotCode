package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    private TalonFX motor = new TalonFX(3, "rio");
    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0.9).withSlot(0);

    public enum ArmLocation {
        INTAKE, OUTTAKE, OUT, UNDEFINED, DEFAULT, GROUND,
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
        motionMagicConfigs.MotionMagicCruiseVelocity = 125; 
        motionMagicConfigs.MotionMagicJerk = 200;

        
        motor.getConfigurator().apply(armSlot0Configs);
        motor.getConfigurator().apply(motionMagicConfigs);

        motor.setPosition(0);
        motor.setNeutralMode(NeutralModeValue.Brake);

        locationsMap.put(ArmLocation.INTAKE, -13.0);
        locationsMap.put(ArmLocation.OUTTAKE, 26.0);
        locationsMap.put(ArmLocation.OUT, 10.0);
        locationsMap.put(ArmLocation.DEFAULT, 0.0);
        locationsMap.put(ArmLocation.GROUND, 63.5);

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

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm/Motor/EncoderPos", motor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Arm/Motor/MotorVoltage", motor.getMotorVoltage().getValueAsDouble());
    }
}
