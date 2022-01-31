package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Climber extends SubsystemBase {
    /**
 * @author Aahana Shrivastava
 */
    private WPI_TalonFX leftElevator, rightElevator;

    public Climber(WPI_TalonFX left, WPI_TalonFX right) {
        leftElevator = left;
        leftElevator.setInverted(true);
        leftElevator.setNeutralMode(NeutralMode.Brake);

        rightElevator = right;
        rightElevator.setInverted(true);
        rightElevator.setNeutralMode(NeutralMode.Brake);

    
        
  
    
    }

    public void setSpeed(double asDouble) {
    }
}
