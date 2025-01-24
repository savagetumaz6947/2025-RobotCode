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

    public enum elevatorPosition {
        Bottom, Mid, Top
    }

    final PositionVoltage positionVoltage = new PositionVoltage(1).withSlot(0);
    
    public Elevator() {
        Slot0Configs elevatorSlot0Configs = new Slot0Configs();
        elevatorSlot0Configs.kP = 1;
        elevatorSlot0Configs.kI = 0;
        elevatorSlot0Configs.kD = 0;
        
        left.getConfigurator().apply(elevatorSlot0Configs);
        left.setPosition(0);

        right.setControl(new Follower(1, false));
        left.setPosition(0);
    }

    public Command set(DoubleSupplier voltage){
        return this.run(() -> {
            if ((left.getPosition().getValueAsDouble() <= 0 && voltage.getAsDouble() <= 0) || 
                (left.getPosition().getValueAsDouble() >= 99999 && voltage.getAsDouble() >= 0)) {
                left.setVoltage(getFeedForward());
            } else {
                left.setVoltage(voltage.getAsDouble() * 2 + getFeedForward());
            }
        });
    }

    public Command setPosition(elevatorPosition position){
        return this.run(() ->{
            if (position == elevatorPosition.Bottom) {
                left.setControl(positionVoltage.withPosition(0));
            }
            else if (position == elevatorPosition.Mid) {
                left.setControl(positionVoltage.withPosition(1));
            }
            else if (position == elevatorPosition.Top) {
                left.setControl(positionVoltage.withPosition(2));
            }
        });
    }

    public double getFeedForward() {
        return 0.24;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator/elevator/Volt", left.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Elevator/elevator/encoder", left.getPosition().getValueAsDouble());
    }

}
