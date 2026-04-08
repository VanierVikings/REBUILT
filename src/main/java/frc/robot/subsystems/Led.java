package frc.robot.subsystems;
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
        LEDPattern white = LEDpattern.solid(Color.kwhite)
        LEDPattern green = LEDpattern.solid(Color.kgreen)
        


          // all hues at maximum saturation and half brightness
        private final LEDPattern m_rainbow = LEDPattern.rainbow(255, 128);

        // Our LED strip has a density of 120 LEDs per meter
        private static final Distance kLedSpacing = Meters.of(1 / 120.0);


    }
   
    
    
    public enum LEDState {
        startShootwhite,
        readyGreen,
        test,
        Disabled
    }

    switch(newState){
        case startShootwhite:
            white.applyTo(m_ledBuffer);
            m_led.setData(m_ledBuffer);
            break;
        
        case readyGreen:
            green.applyTo(m_ledBuffer);
            m_led.setData(m_ledBuffer);
            break;
        
        case test:
            m_rainbow.applyTo(m_ledBuffer);
            m_led.setData(m_ledBuffer);
            //blue.applyTo(m_ledBuffer);
            //m_led.setData(m_ledBuffer);
            break;


    }
}