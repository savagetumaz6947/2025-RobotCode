package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Climber extends SubsystemBase {
    private TalonFX motor = new TalonFX(24, "rio");
    private ClimberLocation state = ClimberLocation.IN;

    public enum ClimberLocation{
        OUT, IN, UNDEFINED
    }

    private Map<ClimberLocation, Double> locationsMap = new HashMap<>();

    public Climber() {
        motor.setNeutralMode(NeutralModeValue.Brake);
        locationsMap.put(ClimberLocation.OUT, 15.0);
        locationsMap.put(ClimberLocation.IN, 0.0);
    }

    public Command set(ClimberLocation location) {
        state = location;
        return this.run(() -> {
            motor.setVoltage(locationsMap.get(location));
        }).until(() -> MathUtil.isNear(locationsMap.get(location), motor.getPosition().getValueAsDouble(), 1));
    }

    public Command setVoltageCommand(DoubleSupplier voltage) {
        return this.run(() -> motor.setVoltage(voltage.getAsDouble() * 12));
    }

    public ClimberLocation getState() {
        return state;
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
