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
//------------------------------------------------------------------------------------------------------------------------
public class AlgaePivot extends SubsystemBase {
    private TalonFX motor = new TalonFX(22, "rio");
    final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(1).withSlot(0);
//------------------------------------------------------------------------------------------------------------------------
    public enum AlgaePivotLocation {
        INTAKE, DEFAULT, UNDEFINED
    }

    private Map<AlgaePivotLocation, Double> locationsMap = new HashMap<>();

    private AlgaePivotLocation state = AlgaePivotLocation.INTAKE;

    public AlgaePivot() {
        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicAcceleration = 30;
        motionMagicConfigs.MotionMagicCruiseVelocity = 70;

        Slot0Configs pivotSlot0Configs = new Slot0Configs();
        pivotSlot0Configs.kP = 0.5;
        pivotSlot0Configs.kI = 0;
        pivotSlot0Configs.kD = 0;
        pivotSlot0Configs.kV = 0.1;
        pivotSlot0Configs.kA = 0.01;

        motor.getConfigurator().apply(pivotSlot0Configs);
        motor.getConfigurator().apply(motionMagicConfigs);
        motor.setNeutralMode(NeutralModeValue.Brake);
        motor.setPosition(0);

        locationsMap.put(AlgaePivotLocation.INTAKE, -5.5);
        locationsMap.put(AlgaePivotLocation.DEFAULT, 0.0);

        this.setDefaultCommand(this.set(() -> 0).repeatedly());
    }
//------------------------------------------------------------------------------------------------------------------------
    public Command set(DoubleSupplier voltage) {
        if (voltage.getAsDouble() != 0) state = AlgaePivotLocation.UNDEFINED;
        return this.run(() -> {
            motor.setVoltage(voltage.getAsDouble());
        });
    }
//------------------------------------------------------------------------------------------------------------------------
    public Command set(AlgaePivotLocation location) {
        return this.run(() -> {
            motor.setControl(motionMagicRequest.withPosition(locationsMap.get(location)).withFeedForward(getFeedForward()));
            state = location;
        }).until(() -> MathUtil.isNear(locationsMap.get(location), motor.getPosition().getValueAsDouble(), 0.25));
    }

    public double getFeedForward() {
        double kG = 0.4;
        double currentPosition = motor.getPosition().getValueAsDouble(); 
        if (MathUtil.isNear(currentPosition, -5.5, 1.0)) 
        {  
            return kG;  
        } else 

        {
            return 0; // 其他位置不補償
        }
    }
//------------------------------------------------------------------------------------------------------------------------
    public AlgaePivotLocation getState() {
        return state;
    }

    public Command eStop() {
        return this.runOnce(() -> {
            motor.set(0);
            state = AlgaePivotLocation.UNDEFINED;
        });
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("AlgaePivot/Motor/EncoderPos", motor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("AlgaePivot/Motor/FeedForward", getFeedForward());
    }
}
