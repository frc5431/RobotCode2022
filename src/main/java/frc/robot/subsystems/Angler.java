package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Systems;
import edu.wpi.first.wpilibj.Servo;

public class Angler extends SubsystemBase {
/**
     * @author Colin Wong
     * @author Aahana Shrivastava
     * @author Addison Pardini
     */
    private Servo anglerServo;
    
    public Angler(Servo servo) {
        anglerServo = servo;
    }

    public Angler(Systems systems, boolean b) {
    }

    public void set(double value) {
        anglerServo.set(value);
    }
}
//angler wired in pwm 0