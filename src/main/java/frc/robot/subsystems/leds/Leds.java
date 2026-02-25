package frc.robot.subsystems.leds;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Map;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.lib.helpers.LedHelper;
import frc.robot.subsystems.SuperStructure;

public class Leds extends SubsystemBase {
    private final int LED_PORT = 8;
    private final int LED_BUFFER_LENGTH = 19;

    private final AddressableLED led = new AddressableLED(LED_PORT);
    private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(LED_BUFFER_LENGTH);

    public Leds() {
        this.led.setLength(LED_BUFFER_LENGTH);
        this.led.setData(this.buffer);
        this.led.start();
    }

    public void defaultMode() {
        LEDPattern base = LEDPattern.gradient(GradientType.kDiscontinuous, LedHelper.Aque.color);
        LEDPattern pattern = base.breathe(Seconds.of(1.5));
        pattern.applyTo(this.buffer);   
    }

    public void flashRed() {
        LEDPattern base = LEDPattern.gradient(GradientType.kContinuous, LedHelper.Red.color);
        LEDPattern pattern = base.blink(Seconds.of(0.08));
        pattern.applyTo(this.buffer);
    }

    public void solidBlue() {
        LEDPattern pattern = LEDPattern.solid(LedHelper.BlueAlliance.color);
        pattern.applyTo(this.buffer);
    }

    public void solidRed() {
        LEDPattern pattern = LEDPattern.solid(LedHelper.Red.color);
        pattern.applyTo(this.buffer);
    }

    public void flashGreen() {
        LEDPattern base = LEDPattern.gradient(GradientType.kContinuous, LedHelper.Green.color);
        LEDPattern pattern = base.blink(Seconds.of(0.08));
        pattern.applyTo(this.buffer);
    }

    @Override
    public void periodic() {
        if (DriverStation.isEStopped()) {
            this.flashRed();
        } else if (SuperStructure.getInstance().inputs.wantScore) {
            this.flashGreen();
        } else if (DriverStation.isFMSAttached() & DriverStation.isDisabled()) {
            if (!Robot.isRedAlliance.get())
                this.solidBlue();
            else
                this.solidRed();
        } else {
            this.defaultMode();
        }

        this.led.setData(this.buffer);
    }
}
