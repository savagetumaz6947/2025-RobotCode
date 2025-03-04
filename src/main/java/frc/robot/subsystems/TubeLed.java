package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TubeLed extends SubsystemBase {
    private final DigitalOutput green = new DigitalOutput(9);

    public enum LedMode {
        ON, OFF, FLASH
    }

    private static LedMode mode = LedMode.ON;

    public static void setMode(LedMode modeSet) {
        mode = modeSet;
    }

    @Override
    public void periodic() {
        if (mode == LedMode.FLASH || RobotState.isDisabled()) {
            green.set(!green.get());
        } else if (mode == LedMode.ON) {
            green.set(false);
        } else if (mode == LedMode.OFF) {
            green.set(true);
        }
    }
}
