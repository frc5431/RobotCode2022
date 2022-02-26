package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Climber extends SubsystemBase {
    /**
     * @author Aahana Shrivastava
     */
    private WPI_TalonFX climberExtend, climberHinge;

    public Climber(WPI_TalonFX climberExtend, WPI_TalonFX climberHinge) {
        this.climberExtend = climberExtend;
        this.climberExtend.setInverted(true);
        this.climberExtend.setNeutralMode(NeutralMode.Brake);

        this.climberHinge = climberHinge;
        this.climberHinge.setInverted(true);
        this.climberHinge.setNeutralMode(NeutralMode.Brake);
    }

    public void setSpeed(double asDouble) {
    }
}
