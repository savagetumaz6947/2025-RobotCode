package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pivot extends SubsystemBase {
    private SparkMax intake = new SparkMax(51, MotorType.kBrushless);
    private TalonFX pivot = new TalonFX(4, "rio");
    final PositionVoltage request = new PositionVoltage(3.75).withSlot(0);

    public enum PivotLocation {
        INTAKE, OUTTAKE, UNDEFINED
    }

    private PivotLocation state = PivotLocation.INTAKE;

    public Pivot () {
        Slot0Configs pivotSlot0Configs = new Slot0Configs();
        pivotSlot0Configs.kP = 1.05;
        pivotSlot0Configs.kI = 0.35;
        pivotSlot0Configs.kD = 0.085;

        pivot.getConfigurator().apply(pivotSlot0Configs);
        pivot.setPosition(0);

        SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
        sparkMaxConfig.smartCurrentLimit(10, 10);
        intake.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        this.setDefaultCommand(this.run(() -> {
            intake.setVoltage(0.5);
        }));
    }

    public Command set(PivotLocation location) {
        return this.run(() -> {
            if (location == PivotLocation.INTAKE){
                pivot.setControl(request.withPosition(0));
            }
            else if (location == PivotLocation.OUTTAKE){
                pivot.setControl(request.withPosition(3.75));
            }
            state = location;
        });
    }

    public PivotLocation getState() {
        return state;
    }

    public Command in() {
        return this.run(() -> {
            intake.setVoltage(8);
        }).repeatedly();
    }

    public Command out(){
        return this.run(() -> {
            intake.setVoltage(-2);
        }).repeatedly();
    }

    public Command eStop() {
        return this.run(() -> {
            intake.setVoltage(0);
            pivot.setVoltage(0);
            state = PivotLocation.UNDEFINED;
        });
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pivot/Intake/AppliedOutput", intake.getBusVoltage() * intake.getAppliedOutput());
        SmartDashboard.putNumber("Pivot/Intake/OutputCurrent", intake.getOutputCurrent());
        SmartDashboard.putNumber("Pivot/Pivot/Encoder", pivot.getPosition().getValueAsDouble());
    }
}
