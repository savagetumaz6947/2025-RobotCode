package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    private TalonFX arm = new TalonFX(3, "rio");
    private final PositionVoltage request = new PositionVoltage(0.9).withSlot(0);

    public enum ArmLocation{
        intake, outtake;
    }

    public Arm () {
        Slot0Configs armSlot0Configs = new Slot0Configs();
        armSlot0Configs.kP = 0.6;
        armSlot0Configs.kI = 0.15;
        armSlot0Configs.kD = 0.085;
        arm.getConfigurator().apply(armSlot0Configs);
        arm.setPosition(0);
    }

    public Command turn (DoubleSupplier volt){
        return this.run(() -> {
            arm.setVoltage(volt.getAsDouble()*1 + getFeedForward());
        });
    }

    public Command turn (ArmLocation location){
        return this.run(() -> {
            if (location == ArmLocation.intake) {
                arm.setControl(request.withPosition(0));
            }
            else if (location == ArmLocation.outtake) {
                arm.setControl(request.withPosition(degreeToEncoder(25)).withFeedForward(getFeedForward()));
            }
        });
    }

    public double getFeedForward() {
        return getAbsoluteDegrees() * -3.35e-3 - 0.139;
    }

    private double getAbsoluteDegrees() {
        return arm.getPosition().getValueAsDouble() * 11.09 - 16;
    }

    private double degreeToEncoder(double degree) {
        return (degree + 16)/11.09;
    }

    public Command eStop() {
        return this.run(() -> {
            arm.setVoltage(0);
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