package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class Intake extends SubsystemBase {
    private WPI_TalonFX intakeMotor1;

    public Intake(WPI_TalonFX motor) {
        intakeMotor1 = motor;
        intakeMotor1.setInverted(true);
        intakeMotor1.setNeutralMode(NeutralMode.Brake);

    }

    public void setSpeed(double speed) {
        intakeMotor1.set(ControlMode.PercentOutput, speed);
    }
}
