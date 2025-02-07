package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class algaeIntake extends SubsystemBase {
    private TalonFX motor = new TalonFX(6, "rio");

    public enum algaeIntakeState{
        DEFAULT, IN, OUT
    }

    private Map<algaeIntakeState, Double> stateMap = new HashMap<>();
    private algaeIntakeState state = algaeIntakeState.DEFAULT;

    public algaeIntake (){
        stateMap.put(algaeIntakeState.DEFAULT, 0.0);
        stateMap.put(algaeIntakeState.IN, 12.0);
        stateMap.put(algaeIntakeState.OUT, -3.0);

        this.setDefaultCommand(this.set(algaeIntakeState.DEFAULT).repeatedly());
    }

    public Command set(algaeIntakeState state){
        return this.runOnce(() -> {
            motor.setVoltage(stateMap.get(state));
            this.state = state;
        });
    }

    public algaeIntakeState getState() {
        return state;
    }

    public Command eStop() {
        return this.runOnce(() -> {
            motor.setVoltage(0);
            state = algaeIntakeState.DEFAULT;
        });
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("algaeIntake/Motor/EncoderPos", motor.getPosition().getValueAsDouble());
    }
}
