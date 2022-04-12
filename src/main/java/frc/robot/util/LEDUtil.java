package frc.robot.util;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class LEDUtil {
    public static void setColor(AddressableLED led, Color color) {
        AddressableLEDBuffer buffer = new AddressableLEDBuffer(120);

        for (int i = 0; i < buffer.getLength(); i++)
            buffer.setLED(i, color);
        
        led.setData(buffer);
    }
}
