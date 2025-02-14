package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private TalonFX motor = new TalonFX(6, "rio");

    public enum ClimberState {
        IN, OUT
    }

    private Map<ClimberState, Double> stateMap = new HashMap<>();
    private ClimberState state = ClimberState.IN;

    public Climber(){

        stateMap.put(ClimberState.IN, 0.0);
        stateMap.put(ClimberState.OUT, 1.0);

        this.setDefaultCommand(this.set(ClimberState.IN).repeatedly());
    }

    public Command set(ClimberState state){
        return this.runOnce(() -> {
            motor.setVoltage(stateMap.get(state));
            this.state = state;
        });
    }

    public ClimberState getState() {
        return state;
    }

    public Command eStop() {
        return this.runOnce(() -> {
            motor.set(0);
            state = ClimberState.IN;
        });
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("climber/Motor/EncoderPos", motor.getPosition().getValueAsDouble());
    }
}
