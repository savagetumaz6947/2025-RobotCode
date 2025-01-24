package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private SparkMax intake = new SparkMax(51, MotorType.kBrushless);

    public enum IntakeState {
        IN, OUT, DEFAULT
    };

    private Map<IntakeState, Double> stateMap = new HashMap<>();

    private IntakeState state = IntakeState.DEFAULT;

    public Intake() {
        SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
        sparkMaxConfig.smartCurrentLimit(10, 10);
        intake.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        stateMap.put(IntakeState.DEFAULT, 0.5);
        stateMap.put(IntakeState.IN, 8.0);
        stateMap.put(IntakeState.OUT, -2.0);
        
        this.setDefaultCommand(this.set(IntakeState.DEFAULT).repeatedly());
    }

    public Command set(IntakeState state) {
        return this.runOnce(() -> {
            intake.setVoltage(stateMap.get(state));
            this.state = state;
        });
    }

    public Command eStop() {
        return this.runOnce(() -> {
            intake.setVoltage(0);
            this.state = IntakeState.DEFAULT;
        });
    }

    public IntakeState getState() {
        return state;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake/AppliedOutput", intake.getBusVoltage() * intake.getAppliedOutput());
        SmartDashboard.putNumber("Intake/OutputCurrent", intake.getOutputCurrent());
    }
}
