package frc.robot.subsystems.leds;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Map;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.helpers.LedHelper;
import frc.robot.subsystems.SuperStructure;

public class Leds extends SubsystemBase {
    private final int LED_PORT = 0;
    private final int LED_BUFFER_LENGTH = 19; // TODO

    private final AddressableLED led = new AddressableLED(LED_PORT);
    private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(LED_BUFFER_LENGTH);

    public Leds() {
        this.led.setLength(LED_BUFFER_LENGTH);
        this.led.setData(this.buffer);
        this.led.start();
    }

    public void defaultMode() {
        // TODO
        Map<Double, Color> maskSteps = Map.of(0.0, LedHelper.White.color, 0.2, LedHelper.Black.color);
        LEDPattern base = LEDPattern.solid(LedHelper.Orange.color);
        LEDPattern mask = LEDPattern.steps(maskSteps).scrollAtRelativeSpeed(
                Percent.per(Second).of(75.0));
        LEDPattern pattern = base.mask(mask);
        pattern.applyTo(this.buffer);
    }

    public void flashRed() {
        LEDPattern base = LEDPattern.gradient(GradientType.kContinuous, LedHelper.Red.color);
        LEDPattern pattern = base.blink(Seconds.of(0.1));
        pattern.applyTo(this.buffer);
    }

    public void solidRed() {
        LEDPattern red = LEDPattern.solid(LedHelper.Red.color);
        red.applyTo(this.buffer);
    }

    public void flashGreen() {
        LEDPattern base = LEDPattern.gradient(GradientType.kContinuous, LedHelper.Green.color);
        LEDPattern pattern = base.blink(Seconds.of(0.08));
        pattern.applyTo(this.buffer);
    }

    // private void rainbow() {
    //     LEDPattern rainbow = LEDPattern.rainbow(255, 128);
    //     Distance spacing = Meters.of(1.0 / 120.0);
    //     LEDPattern pattern = rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), spacing);
    //     pattern.applyTo(this.buffer);
    // }

    @Override
    public void periodic() {
        if (DriverStation.isEStopped())
            this.solidRed();
        else if (DriverStation.isDisabled())
            this.defaultMode();
        else if (SuperStructure.getInstance().inputs.wantScore)
            this.flashGreen();
        else this.defaultMode();

        this.led.setData(this.buffer);
    }
}
