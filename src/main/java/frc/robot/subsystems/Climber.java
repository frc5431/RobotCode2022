package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Climber extends SubsystemBase {
    /**
 * @author Aahana Shrivastava
 */
    private WPI_TalonFX leftElevator1, leftElevator2, rightElevator1, rightElevator2;

    public Climber(WPI_TalonFX le1, WPI_TalonFX le2, WPI_TalonFX re1, WPI_TalonFX re2) {
        leftElevator1 = le1;
        leftElevator1.setInverted(true);
        leftElevator1.setNeutralMode(NeutralMode.Brake);

        leftElevator2 = le2;
        leftElevator2.setInverted(true);
        leftElevator2.setNeutralMode(NeutralMode.Brake);

        rightElevator1 = re1;
        rightElevator1.setInverted(true);
        rightElevator1.setNeutralMode(NeutralMode.Brake);

        rightElevator2 = re2;  
        rightElevator2.setInverted(true);
        rightElevator2.setNeutralMode(NeutralMode.Brake);

        
        
  
    
    }

    public void setSpeed(double asDouble) {
    }
}
