package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;

/**
 * @author Colin Wong
 * @author Aahana Shrivastava
 * @author Addison Pardini
 */
public class Angler extends SubsystemBase {

    public static final double DEFAULT_SPEED = 0.025;
    public static final double DOWN_LIMIT = 0.225; // 0.2
    public static final double UP_LIMIT = 0.8; // 0.7

    private Servo anglerServo;
    
    public Angler(Servo servo) {
        anglerServo = servo;

        Constants.tab_subsystems.addNumber("Angler Position", () -> anglerServo.get())
                .withPosition(18, 2)
                .withSize(2, 1);
        Constants.tab_subsystems.addNumber("Angler Raw", () -> anglerServo.getRaw())
                .withPosition(18, 1)
                .withSize(2, 1);
    }

    public void set(double value) {
        anglerServo.set(MathUtil.clamp(value, DOWN_LIMIT, UP_LIMIT));
    }

    public void change(double value) {
        set(get() + value);
    }

    public double get() {
        return anglerServo.get();
    }
}
//angler wired in pwm 0