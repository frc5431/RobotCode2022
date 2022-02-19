package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class Intake extends SubsystemBase {

    public static final double DEFAULT_SPEED = 1.0;
    public static final boolean REVERSE = true;
    public static final NeutralMode NEUTRALMODE = NeutralMode.Coast;
    public static final double RAMPING_FROM_0_TO_FULL = 0.5;

    private WPI_TalonFX intakeMotor;

    public Intake(WPI_TalonFX motor) {
        intakeMotor = motor;
        intakeMotor.setInverted(REVERSE);
        intakeMotor.setNeutralMode(NEUTRALMODE);

        intakeMotor.configOpenloopRamp(RAMPING_FROM_0_TO_FULL);
    }

    public void setSpeed(double speed) {
        intakeMotor.set(ControlMode.PercentOutput, speed);
    }
}
