package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class Elevator extends SubsystemBase {
    private TalonFX left = new TalonFX(25, "rio");
    private TalonFX right = new TalonFX(26, "rio");

    final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(1).withSlot(0);

    public enum ElevatorLocation {
        BOTTOM, MID, TOP, UNDEFINED, ALGAE, SOURCE, DOWNINTAKE
    }

    private static Map<ElevatorLocation, Double> locationsMap = new HashMap<>();
    private ElevatorLocation state = ElevatorLocation.BOTTOM;

    private static ElevatorSim sim;
    public static MechanismLigament2d mech2d;

    public Elevator() {
        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicAcceleration = 100;
        motionMagicConfigs.MotionMagicCruiseVelocity = 150;
        motionMagicConfigs.MotionMagicJerk = 500;
       
        Slot0Configs elevatorSlot0Configs = new Slot0Configs();
        elevatorSlot0Configs.kP = 0.8;
        elevatorSlot0Configs.kI = 0.0;
        elevatorSlot0Configs.kD = 0.25;
        elevatorSlot0Configs.kV = 0.1; 
        elevatorSlot0Configs.kA = 0.01; 
        elevatorSlot0Configs.kS = 0.1; 
        
        left.getConfigurator().apply(elevatorSlot0Configs);
        left.getConfigurator().apply(motionMagicConfigs);
        left.getConfigurator().apply(new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitThreshold(35.0).withForwardSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(0.0).withReverseSoftLimitEnable(true));

        left.setNeutralMode(NeutralModeValue.Brake);
        left.setPosition(0);

        right.setNeutralMode(NeutralModeValue.Brake);
        right.setControl(new Follower(left.getDeviceID(), false));

        locationsMap.put(ElevatorLocation.BOTTOM, 2.0);
        locationsMap.put(ElevatorLocation.MID, 13.5);
        locationsMap.put(ElevatorLocation.TOP, 32.0);
        locationsMap.put(ElevatorLocation.ALGAE, 20.0);
        locationsMap.put(ElevatorLocation.SOURCE, 6.0);
        locationsMap.put(ElevatorLocation.DOWNINTAKE, 5.0);

        this.setDefaultCommand(this.set(() -> 0.0).repeatedly());

        if (Robot.isSimulation()) configureSimulation();

        SmartDashboard.putNumber("X_OFFSET", Constants.ReefSelector.X_OFFSET.in(Meters));
        SmartDashboard.putNumber("LEFT_Y_OFFSET", Constants.ReefSelector.LEFT_Y_OFFSET.in(Meters));
        SmartDashboard.putNumber("RIGHT_Y_OFFSET", Constants.ReefSelector.RIGHT_Y_OFFSET.in(Meters));
    }

    public Command set(DoubleSupplier voltage){
        if (voltage.getAsDouble() != 0) state = ElevatorLocation.UNDEFINED;
        return this.runOnce(() -> {
            left.setVoltage(voltage.getAsDouble() * 1 + getFeedForward());
        });
    }

    public Command set(ElevatorLocation position){
        state = position;
        return this.run(() ->{
            double targetPosition = locationsMap.get(position);
            left.setControl(motionMagicRequest.withPosition(targetPosition).withFeedForward(getFeedForward()));
        }).until(() -> MathUtil.isNear(locationsMap.get(position), left.getPosition().getValueAsDouble(), 3));
    }

    public double getFeedForward() {
        double kG = 0.35;
        double position = left.getPosition().getValueAsDouble();
        if (position < 2.0) {
            return 0.3;
        }
        return (position > 25.0) ? kG + 0.1 : kG;
    }

    public ElevatorLocation getState() {
        return state;
    }

    public double getHeight(){
        return left.getPosition().getValueAsDouble();
    }

    public static double getMaxHeight(){
        return locationsMap.get(ElevatorLocation.TOP);
    }

    public Command eStop() {
        state = ElevatorLocation.UNDEFINED;
        return this.runOnce(() -> {
            left.set(0);
        });
    }

    private void configureSimulation() {
        sim = new ElevatorSim(DCMotor.getKrakenX60(2), 10, 8, 0.061,
                0.01, 1.3, true,
                0.01);
        mech2d = RobotContainer.bigMech2dRoot.append(new MechanismLigament2d("Elevator", sim.getPositionMeters(), 90));
    }
    
    @Override
    public void simulationPeriodic() {
        left.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());
        sim.setInputVoltage(left.getSimState().getMotorVoltage());
        sim.update(0.02);

        mech2d.setLength(sim.getPositionMeters());

        left.getSimState().setRawRotorPosition(Units.radiansToRotations(sim.getPositionMeters() / 0.061 * 10));
        left.getSimState().setRotorVelocity(RadiansPerSecond.of(sim.getVelocityMetersPerSecond() / 0.061 * 10));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator/Left/EncoderPos", left.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Elevator/Left/MotorVoltage", left.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Elevator/Left/MotorCurrent", left.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Elevator/Left/MotorTemperature", left.getDeviceTemp().getValueAsDouble());

        Constants.ReefSelector.X_OFFSET = Meters.of(SmartDashboard.getNumber("X_OFFSET", Constants.ReefSelector.X_OFFSET.in(Meters)));
        Constants.ReefSelector.LEFT_Y_OFFSET = Meters.of(SmartDashboard.getNumber("LEFT_Y_OFFSET", Constants.ReefSelector.LEFT_Y_OFFSET.in(Meters)));
        Constants.ReefSelector.RIGHT_Y_OFFSET = Meters.of(SmartDashboard.getNumber("RIGHT_Y_OFFSET", Constants.ReefSelector.RIGHT_Y_OFFSET.in(Meters)));
    }
}
