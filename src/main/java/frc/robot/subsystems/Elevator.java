package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    private TalonFX left = new TalonFX(1, "rio");
    private TalonFX right = new TalonFX(2, "rio");
    final PositionVoltage request = new PositionVoltage(1).withSlot(0);

    public enum ElevatorPosition {
        BOTTOM, MID, TOP, UNDEFINED
    }

    private ElevatorPosition state = ElevatorPosition.BOTTOM;

    public Elevator() {
        Slot0Configs elevatorSlot0Configs = new Slot0Configs();
        elevatorSlot0Configs.kP = 0.5;
        elevatorSlot0Configs.kI = 0.025;
        elevatorSlot0Configs.kD = 0.2;
        
        left.getConfigurator().apply(elevatorSlot0Configs);
        left.setPosition(0);

        right.setControl(new Follower(1, false));
    }

    public Command set(DoubleSupplier voltage){
        return this.run(() -> {
            if ((left.getPosition().getValueAsDouble() <= 0 && voltage.getAsDouble() <= 0) || 
                (left.getPosition().getValueAsDouble() >= 99999 && voltage.getAsDouble() >= 0)) {
                left.setVoltage(getFeedForward());
            } else {
                left.setVoltage(voltage.getAsDouble() * 2 + getFeedForward());
            }
            state = ElevatorPosition.UNDEFINED;
        });
    }

    public Command set(ElevatorPosition position){
        return this.run(() ->{
            if (position == ElevatorPosition.BOTTOM) {
                left.setControl(request.withPosition(1));
            }
            else if (position == ElevatorPosition.MID) {
                left.setControl(request.withPosition(14.2));
            }
            else if (position == ElevatorPosition.TOP) {
                left.setControl(request.withPosition(33.9));
            }
            state = position;
        });
    }

    public double getFeedForward() {
        return 0.24;
    }

    public ElevatorPosition getState() {
        return state;
    }

    public Command eStop() {
        return this.run(() -> {
            left.set(0);
            state = ElevatorPosition.UNDEFINED;
        });
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator/elevator/Volt", left.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Elevator/elevator/encoder", left.getPosition().getValueAsDouble());
    }

}
