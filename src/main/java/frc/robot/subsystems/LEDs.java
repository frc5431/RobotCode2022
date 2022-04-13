package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
    
    public static final int LED_LENGTH = 120;

    // private AddressableLED led;
    // private AddressableLEDBuffer buffer;

    // public LEDs(AddressableLED led) {
    //     this.led = led;
    //     this.led.setLength(LED_LENGTH);
    //     this.led.start();

    //     this.buffer = new AddressableLEDBuffer(LED_LENGTH);
    // }

    // public void setColor(Color color) {
    //     for (int i = 0; i < buffer.getLength(); i++) {
    //         buffer.setLED(i, color);
    //     }

    //     led.setData(buffer);
    // }

    // public void setRGB(int r, int g, int b) {
    //     for (int i = 0; i < buffer.getLength(); i++) {
    //         buffer.setRGB(i, r, g, b);
    //     }

    //     led.setData(buffer);
    // }

    private Spark spark;

    public LEDs(Spark spark) {
        this.spark = spark;
    }

    public void set(double pwmValue) {
        spark.set(MathUtil.clamp(pwmValue, -1, 1));
    }
}
