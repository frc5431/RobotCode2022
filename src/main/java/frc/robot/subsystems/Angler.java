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

    public static final double DEFAULT_SPEED = 0.05;
    public static final double DOWN_LIMIT = 0.3;
    public static final double UP_LIMIT = 0.8;

    private Servo anglerServo;
    
    public Angler(Servo servo) {
        anglerServo = servo;

        Constants.tab_subsystems.addNumber("Angler Position", 
                () -> anglerServo.get());
        // Constants.tab_subsystems.addNumber("Angler Angle", 
        //         () -> anglerServo.getAngle());
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