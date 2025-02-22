package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedStrip extends SubsystemBase {
    private AddressableLED led = new AddressableLED(9);
    private AddressableLEDBuffer buffer = new AddressableLEDBuffer(35);
    private final LEDPattern rainbow = LEDPattern.rainbow(255, 128);
    private static final Distance ledSpacing = Meters.of(1 / 60.0);
    private final LEDPattern scrollingRainbow = rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(0.5), ledSpacing);
    private Elevator elevator;

    private final LEDPattern pattern = LEDPattern.progressMaskLayer(() -> elevator.getheight() / elevator.getMaxHeight());


    public LedStrip () {
        led.setLength(buffer.getLength());
        led.start();
    }

    @Override
    public void periodic() {
        scrollingRainbow.applyTo(buffer);
        pattern.applyTo(buffer);
        led.setData(buffer);
    }
}