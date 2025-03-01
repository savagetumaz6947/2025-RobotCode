package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeIntake extends SubsystemBase {
    private TalonFX motor = new TalonFX(21, "rio");

    public enum AlgaeIntakeState {
        DEFAULT, IN, OUT
    }

    private Map<AlgaeIntakeState, Double> stateMap = new HashMap<>();
    private AlgaeIntakeState state = AlgaeIntakeState.DEFAULT;

    public AlgaeIntake (){
        motor.setNeutralMode(NeutralModeValue.Brake);

        stateMap.put(AlgaeIntakeState.DEFAULT, -0.5);
        stateMap.put(AlgaeIntakeState.IN, -6.0);
        stateMap.put(AlgaeIntakeState.OUT, 12.0);

        this.setDefaultCommand(this.set(AlgaeIntakeState.DEFAULT).repeatedly());
    }

    public Command set(AlgaeIntakeState state){
        return this.runOnce(() -> {
            motor.setVoltage(stateMap.get(state));
            this.state = state;
        });
    }

    public AlgaeIntakeState getState() {
        return state;
    }

    public Command eStop() {
        return this.runOnce(() -> {
            motor.setVoltage(0);
            state = AlgaeIntakeState.DEFAULT;
        });
    }

}
