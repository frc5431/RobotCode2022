package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Servo;

/**
 * @author Colin Wong
 * @author Aahana Shrivastava
 * @author Addison Pardini
 */
public class Angler extends SubsystemBase {

    public static final double DEFAULT_SPEED = 0.1;

    private Servo anglerServo;
    
    public Angler(Servo servo) {
        anglerServo = servo;

        Constants.tab_subsystems.addNumber("Angler Position", 
                () -> anglerServo.get());
        Constants.tab_subsystems.addNumber("Angler Angle", 
                () -> anglerServo.getAngle());
    }

    public void set(double value) {
        anglerServo.set(value);
    }

    public void change(double value) {
        anglerServo.set(anglerServo.get() + value);
    }

    public double get() {
        return anglerServo.get();
    }
}
//angler wired in pwm 0