package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pivot extends SubsystemBase {
    private TalonFX pivot = new TalonFX(4, "rio");
    final PositionVoltage request = new PositionVoltage(3.75).withSlot(0);

    public enum PivotLocation {
        INTAKE, OUTTAKE, UNDEFINED
    }

    public Map<PivotLocation, Double> locationsMap = new HashMap<>();

    private PivotLocation state = PivotLocation.INTAKE;

    public Pivot () {
        // TODO: Neutral mode: brake

        Slot0Configs pivotSlot0Configs = new Slot0Configs();
        pivotSlot0Configs.kP = 1.05;
        pivotSlot0Configs.kI = 0.35;
        pivotSlot0Configs.kD = 0.085;

        pivot.getConfigurator().apply(pivotSlot0Configs);
        pivot.setPosition(0);

        locationsMap.put(PivotLocation.INTAKE, 0.0);
        locationsMap.put(PivotLocation.OUTTAKE, 3.75);

        this.setDefaultCommand(this.set(PivotLocation.INTAKE).repeatedly());
    }

    public Command set(PivotLocation location) {
        return this.run(() -> {
            pivot.setControl(request.withPosition(locationsMap.get(location)));
            state = location;
        }).until(() -> {
            return MathUtil.isNear(locationsMap.get(location), pivot.getPosition().getValueAsDouble(), 0.05);
        });
    }

    public PivotLocation getState() {
        return state;
    }

    public Command eStop() {
        return this.runOnce(() -> {
            pivot.setVoltage(0);
            state = PivotLocation.UNDEFINED;
        });
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pivot/Pivot/Encoder", pivot.getPosition().getValueAsDouble());
    }
}
