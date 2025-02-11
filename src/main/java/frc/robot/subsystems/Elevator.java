package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    private TalonFX left = new TalonFX(1, "rio");
    private TalonFX right = new TalonFX(2, "rio");

    final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(1).withSlot(0);

    public enum ElevatorLocation {
        BOTTOM, MID, TOP, UNDEFINED, ALGAE
    }

    private Map<ElevatorLocation, Double> locationsMap = new HashMap<>();
    private ElevatorLocation state = ElevatorLocation.BOTTOM;

    public Elevator() {
        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicAcceleration = 70;
        motionMagicConfigs.MotionMagicCruiseVelocity = 100;
        motionMagicConfigs.MotionMagicJerk = 500;
       
        Slot0Configs elevatorSlot0Configs = new Slot0Configs();
        elevatorSlot0Configs.kP = 0.8;
        elevatorSlot0Configs.kI = 0.15;
        elevatorSlot0Configs.kD = 0.25;
        elevatorSlot0Configs.kV = 0.1; 
        elevatorSlot0Configs.kA = 0.01; 
        
        left.getConfigurator().apply(elevatorSlot0Configs);
        left.getConfigurator().apply(motionMagicConfigs);
        left.getConfigurator().apply(new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitThreshold(35.0).withForwardSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(0.0).withReverseSoftLimitEnable(true));

        left.setNeutralMode(NeutralModeValue.Brake);
        left.setPosition(0);

        right.setNeutralMode(NeutralModeValue.Brake);
        right.setControl(new Follower(1, false));

        locationsMap.put(ElevatorLocation.BOTTOM, 2.0);
        locationsMap.put(ElevatorLocation.MID, 14.5);
        locationsMap.put(ElevatorLocation.TOP, 34.0);
        locationsMap.put(ElevatorLocation.ALGAE, 20.0);

        this.setDefaultCommand(this.set(() -> 0.0).repeatedly());
    }

    public Command set(DoubleSupplier voltage){
        return this.runOnce(() -> {
            left.setVoltage(voltage.getAsDouble() * 1 + getFeedForward());
            if (voltage.getAsDouble() != 0) state = ElevatorLocation.UNDEFINED;
        });
    }

    public Command set(ElevatorLocation position){
        return this.run(() ->{
            double targetPosition = locationsMap.get(position);
            left.setControl(motionMagicRequest.withPosition(targetPosition).withFeedForward(getFeedForward()));
            state = position;
        }).until(() -> MathUtil.isNear(locationsMap.get(position), left.getPosition().getValueAsDouble(), 3));
    }

    public double getFeedForward() {
        double kG = 0.35;
        double position = left.getPosition().getValueAsDouble();
        if (position < 2.0) {
            return 0.15;
        }
        return (position > 25.0) ? kG + 0.1 : kG;
    }

    public ElevatorLocation getState() {
        return state;
    }

    public Command eStop() {
        return this.runOnce(() -> {
            left.set(0);
            state = ElevatorLocation.UNDEFINED;
        });
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator/Left/EncoderPos", left.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Elevator/Left/MotorVoltage", left.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Elevator/Left/MotorCurrent", left.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Elevator/Left/MotorTemperature", left.getDeviceTemp().getValueAsDouble());
    }
}
