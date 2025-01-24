package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    private TalonFX arm = new TalonFX(3, "rio");
    private final PositionVoltage request = new PositionVoltage(0.9).withSlot(0);

    public enum ArmLocation{
        INTAKE, OUTTAKE, UNDEFINED
    }

    public Map<ArmLocation, Double> locationsMap = new HashMap<>();

    private ArmLocation state = ArmLocation.INTAKE;

    public Arm () {
        Slot0Configs armSlot0Configs = new Slot0Configs();
        armSlot0Configs.kP = 0.6;
        armSlot0Configs.kI = 0.15;
        armSlot0Configs.kD = 0.085;
        arm.getConfigurator().apply(armSlot0Configs);
        arm.setPosition(0);

        locationsMap.put(ArmLocation.INTAKE, degreeToEncoder(-32));
        locationsMap.put(ArmLocation.OUTTAKE, degreeToEncoder(25));

        // this.setDefaultCommand(this.set(() -> 0.0));
    }

    public Command set(DoubleSupplier volt){
        return this.runOnce(() -> {
            arm.setVoltage(volt.getAsDouble()*1 + getFeedForward());
            state = ArmLocation.UNDEFINED;
        });
    }

    public Command set(ArmLocation location){
        return this.run(() -> {
            arm.setControl(request.withPosition(locationsMap.get(location)).withFeedForward(getFeedForward()));
            state = location;
        }).until(() -> {
            return MathUtil.isNear(locationsMap.get(location), arm.getPosition().getValueAsDouble(), 0.05);
        });
    }

    public double getFeedForward() {
        return Math.cos(Math.toRadians(90 - getAbsoluteDegrees())) * -0.42;
    }

    private double getAbsoluteDegrees() {
        return arm.getPosition().getValueAsDouble() * 11.09 - 32;
    }

    private double degreeToEncoder(double degree) {
        return (degree + 32)/11.09;
    }

    public ArmLocation getState() {
        return state;
    }

    public Command eStop() {
        return this.runOnce(() -> {
            arm.setVoltage(0);
            state = ArmLocation.UNDEFINED;
        });
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm/arm/Encoder", arm.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Arm/arm/absoluteDegree", getAbsoluteDegrees());
        SmartDashboard.putNumber("Arm/arm/volt", arm.getMotorVoltage().getValueAsDouble());
    }
}

//0.09