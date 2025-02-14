package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Intake extends SubsystemBase {
    private SparkMax motor = new SparkMax(51, MotorType.kBrushless);
    private SparkMaxSim motorSim;

    public enum IntakeState {
        IN, OUT, DEFAULT
    };

    private Map<IntakeState, Double> stateMap = new HashMap<>();

    private IntakeState state = IntakeState.DEFAULT;

    private static FlywheelSim sim;
    private static MechanismLigament2d mech2d;

    public Intake() {
        SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
        sparkMaxConfig.smartCurrentLimit(10, 10);
        motor.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        stateMap.put(IntakeState.DEFAULT, 0.5);
        stateMap.put(IntakeState.IN, 12.0);
        stateMap.put(IntakeState.OUT, -2.0);
        
        this.setDefaultCommand(this.set(IntakeState.DEFAULT).repeatedly());

        if (Robot.isSimulation()) configureSimulation();
    }

    public Command set(IntakeState state) {
        this.state = state;
        return this.runOnce(() -> {
            motor.setVoltage(stateMap.get(state));
        });
    }

    public IntakeState getState() {
        return state;
    }

    public Command eStop() {
        this.state = IntakeState.DEFAULT;
        return this.runOnce(() -> {
            motor.setVoltage(0);
        });
    }

    private void configureSimulation() {
        motorSim = new SparkMaxSim(motor, DCMotor.getNeo550(1));

        sim = new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getNeo550(1), 0.01, 15), DCMotor.getNeo550(1));
        mech2d = Pivot.mech2d.append(new MechanismLigament2d("Intake", 0.2, 0, 10, new Color8Bit(Color.kBeige)));
    }

    @Override
    public void simulationPeriodic() {
        sim.setInputVoltage(motorSim.getAppliedOutput() * RobotController.getBatteryVoltage());
        sim.update(0.02);

        motorSim.iterate(Units.radiansPerSecondToRotationsPerMinute(sim.getAngularVelocityRadPerSec() * 15), RobotController.getBatteryVoltage(), 0.02);

        mech2d.setAngle(Rotation2d.fromRotations(motor.getEncoder().getPosition() / 15));
    }

    @Override
    public void periodic() {
        
    }
}
