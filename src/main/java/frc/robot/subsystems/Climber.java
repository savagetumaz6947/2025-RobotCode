package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private TalonFX motor = new TalonFX(24, "rio");

    public Climber() {
        motor.setNeutralMode(NeutralModeValue.Brake);
    }

    public Command setVoltageCommand(DoubleSupplier voltage) {
        return this.run(() -> motor.setVoltage(voltage.getAsDouble() * 12));
    }

    public Command eStop() {
        return this.runOnce(() -> {
            motor.setVoltage(0);
        });
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("climber/Motor/EncoderPos", motor.getPosition().getValueAsDouble());
    }
}
