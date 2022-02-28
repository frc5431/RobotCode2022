package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Servo;

public class Angler extends SubsystemBase {
    Servo anglerServo = new Servo(1);

    anglerServo.set(0.5);
    anglerServo.setAngle(75);

    
}
