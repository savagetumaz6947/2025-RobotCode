package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class algaeIntake extends SubsystemBase {
    private TalonFX motor = new TalonFX(1, "rio");

    public enum IntakeState{
        DEFAULT, IN, OUT
    }

    private Map<IntakeState, Double> stateMap = new HashMap<>();
    private IntakeState state = IntakeState.DEFAULT;

    public algaeIntake (){
        stateMap.put(IntakeState.DEFAULT, 0.0);
        stateMap.put(IntakeState.IN, 0.0);
        stateMap.put(IntakeState.IN, 0.0);

        this.setDefaultCommand(this.set(IntakeState.DEFAULT).repeatedly());
    }

    public Command set(IntakeState state){
        return this.runOnce(() -> {
            motor.setVoltage(stateMap.get(state));
            this.state = state;
        });
    }

    public IntakeState getState() {
        return state;
    }

    public Command eStop() {
        return this.runOnce(() -> {
            motor.setVoltage(0);
            state = IntakeState.DEFAULT;
        });
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Arm/Motor/EncoderPos", motor.getPosition().getValueAsDouble());
    }
}
