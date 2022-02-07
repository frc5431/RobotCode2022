package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class Intake extends SubsystemBase {
    private WPI_TalonFX intakeMotor1, intakeMotor2;

    public Intake(WPI_TalonFX motor, WPI_TalonFX motor2) {
        intakeMotor1 = motor;
        intakeMotor1.setInverted(true);
        intakeMotor1.setNeutralMode(NeutralMode.Brake);

        intakeMotor2 = motor2;
        intakeMotor2.setInverted(true);
        intakeMotor2.setNeutralMode(NeutralMode.Brake);

    }

    public void setSpeed(double speed) {
        intakeMotor1.set(ControlMode.PercentOutput, speed);
        intakeMotor2.set(ControlMode.PercentOutput, speed);
    }
}
