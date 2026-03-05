package frc.robot;
import static edu.wpi.first.units.Units.Percent;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;    

public class Led extends SubsystemBase {

  
    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_ledBuffer;

    public Led() {
        m_led = new AddressableLED(1);
        m_ledBuffer = new AddressableLEDBuffer(58);

        // Set the LED strip length before starting it
        m_led.setLength(58);
        m_led.start();
    }

    /** Creates a red gradient pattern. */
    public void redPatternCreator() {
        LEDPattern basePattern = LEDPattern.gradient(GradientType.kContinuous, Color.kRed);
       basePattern.applyTo(m_ledBuffer);
       LEDPattern pattern = basePattern.atBrightness(Percent.of(100));
       pattern.applyTo(m_ledBuffer);
        m_led.setData(m_ledBuffer);
    }

    /** Creates a blue gradient pattern. */
    public void bluePatternCreator() {
        LEDPattern blueLedPattern = LEDPattern.gradient(GradientType.kContinuous, Color.kBlue);
        blueLedPattern.applyTo(m_ledBuffer);
        LEDPattern pattern = blueLedPattern.atBrightness(Percent.of(100));
        pattern.applyTo(m_ledBuffer);
        m_led.setData(m_ledBuffer);
    }

    /** Periodically updates the LED strip with the latest buffer data. */
    @Override
    public void periodic() {
       // m_led.setData(m_ledBuffer);
       redPatternCreator();
    }

    /**
     * Creates a command that runs a pattern on the entire LED strip.
     *
     * @param pattern the LED pattern to run
     * @return a command to execute the LED pattern
     */
    public Command runPattern(LEDPattern pattern) {
        return this.run(() -> {
            pattern.applyTo(m_ledBuffer);
            m_led.setData(m_ledBuffer);
        }).withName("LEDPatternCommand");
    }
}