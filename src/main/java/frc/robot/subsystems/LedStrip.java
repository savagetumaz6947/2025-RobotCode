package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedStrip extends SubsystemBase {
    private DoubleSupplier elevatorHeightSupplier;

    private AddressableLED led = new AddressableLED(9);
    private AddressableLEDBuffer buffer = new AddressableLEDBuffer(35);
    private final LEDPattern rainbow = LEDPattern.rainbow(255, 128);
    // private final Distance ledSpacing = Meters.of(1 / 60.0);
    private final LEDPattern elevatorPattern = LEDPattern.progressMaskLayer(() -> elevatorHeightSupplier.getAsDouble() / Elevator.getMaxHeight());
    private final LEDPattern elevatorRainbow = rainbow.mask(elevatorPattern);

    public LedStrip (DoubleSupplier elevatorHeightSupplier) {
        led.setLength(buffer.getLength());
        led.start();
        this.elevatorHeightSupplier = elevatorHeightSupplier;
    }

    @Override
    public void periodic() {
        // scrollingRainbow.applyTo(buffer);
        elevatorRainbow.applyTo(buffer);
        led.setData(buffer);
    }
}