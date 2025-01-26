package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    private TalonFX left = new TalonFX(1, "rio");
    private TalonFX right = new TalonFX(2, "rio");
    final PositionVoltage request = new PositionVoltage(1).withSlot(0);

    public enum ElevatorLocation {
        BOTTOM, MID, TOP, UNDEFINED
    }

    private Map<ElevatorLocation, Double> locationsMap = new HashMap<>();

    private ElevatorLocation state = ElevatorLocation.BOTTOM;

    public Elevator() {
        Slot0Configs elevatorSlot0Configs = new Slot0Configs();
        elevatorSlot0Configs.kP = 0.5;
        elevatorSlot0Configs.kI = 0.025;
        elevatorSlot0Configs.kD = 0.2;
        
        left.getConfigurator().apply(elevatorSlot0Configs);
        left.getConfigurator().apply(new SoftwareLimitSwitchConfigs()
                                            .withForwardSoftLimitThreshold(33.9).withForwardSoftLimitEnable(true)
                                            .withReverseSoftLimitThreshold(0.0).withReverseSoftLimitEnable(true));
        left.setNeutralMode(NeutralModeValue.Brake);
        left.setPosition(0);

        right.setNeutralMode(NeutralModeValue.Brake);
        right.setControl(new Follower(1, false));

        locationsMap.put(ElevatorLocation.BOTTOM, 3.0);
        locationsMap.put(ElevatorLocation.MID, 14.2);
        locationsMap.put(ElevatorLocation.TOP, 33.9);

        this.setDefaultCommand(this.set(() -> 0.0).repeatedly());
    }

    public Command set(DoubleSupplier voltage){
        return this.runOnce(() -> {
            left.setVoltage(voltage.getAsDouble() * 2 + getFeedForward());
            state = ElevatorLocation.UNDEFINED;
        });
    }

    public Command set(ElevatorLocation position){
        return this.run(() ->{
            left.setControl(request.withPosition(locationsMap.get(position)).withFeedForward(getFeedForward()));
            state = position;
        }).until(() -> MathUtil.isNear(locationsMap.get(position), left.getPosition().getValueAsDouble(), 0.05));
    }

    public double getFeedForward() {
        return 0.24;
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
        SmartDashboard.putNumber("Elevator/elevator/Volt", left.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Elevator/elevator/encoder", left.getPosition().getValueAsDouble());
    }

}
