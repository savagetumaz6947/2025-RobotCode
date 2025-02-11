package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class climber extends SubsystemBase {
    private TalonFX motor = new TalonFX(6, "rio");

    public enum climberState {
        IN, OUT
    }

    private Map<climberState, Double> stateMap = new HashMap<>();
    private climberState state = climberState.IN;

    public climber(){

        stateMap.put(climberState.IN, 0.0);
        stateMap.put(climberState.OUT, 1.0);

        this.setDefaultCommand(this.set(climberState.IN).repeatedly());
    }

    public Command set(climberState state){
        return this.runOnce(() -> {
            motor.setVoltage(stateMap.get(state));
            this.state = state;
        });
    }

    public climberState getState() {
        return state;
    }

    public Command eStop() {
        return this.runOnce(() -> {
            motor.set(0);
            state = climberState.IN;
        });
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("climber/Motor/EncoderPos", motor.getPosition().getValueAsDouble());
    }
}
