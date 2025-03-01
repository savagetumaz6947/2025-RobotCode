package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Intake extends SubsystemBase {
    private TalonFX motor = new TalonFX(27, "rio");

    public enum IntakeState {
        ALGAEIN, IN, OUT, DEFAULT, TEST
    };

    private Map<IntakeState, Double> stateMap = new HashMap<>();

    private IntakeState state = IntakeState.DEFAULT;

    private static FlywheelSim sim;
    private static MechanismLigament2d mech2d;

    public Intake() {
        motor.getConfigurator().apply(new CurrentLimitsConfigs().withStatorCurrentLimit(20)
            .withStatorCurrentLimitEnable(true));

        stateMap.put(IntakeState.DEFAULT, 0.5);
        stateMap.put(IntakeState.IN, 4.0);
        stateMap.put(IntakeState.ALGAEIN, 12.0);
        stateMap.put(IntakeState.OUT, -4.0);
        
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
        sim = new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(1), 0.01, 15), DCMotor.getNeo550(1));
        mech2d = Pivot.mech2d.append(new MechanismLigament2d("Intake", 0.2, 0, 10, new Color8Bit(Color.kBeige)));
    }

    @Override
    public void simulationPeriodic() {
        sim.setInputVoltage(motor.getSimState().getMotorVoltage());
        sim.update(0.02);

        motor.getSimState().addRotorPosition(Units.radiansToRotations(sim.getAngularVelocityRadPerSec()) * TimedRobot.kDefaultPeriod * 15);
        motor.getSimState().setRotorVelocity(RPM.of(sim.getAngularVelocityRPM() * 15));

        mech2d.setAngle(Rotation2d.fromRotations(motor.getPosition().getValueAsDouble() / 15));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake/Motor/Current", motor.getStatorCurrent().getValueAsDouble());
    }
}
