package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    public static final double DEFAULT_SPEED = 1.0;
    public static final boolean REVERSE = false;
    public static final IdleMode IDLEMODE = IdleMode.kCoast;
    public static final double RAMPING_FROM_0_TO_FULL = 0.0; // 0.5

    private CANSparkMax intakeMotor, intakeMotor_follow;

    public Intake(CANSparkMax leftMotor, CANSparkMax rightMotor) {
        intakeMotor = leftMotor;
        intakeMotor_follow = rightMotor;
        intakeMotor_follow.follow(intakeMotor, true);
        intakeMotor.setInverted(REVERSE);
        intakeMotor.setIdleMode(IDLEMODE);
        intakeMotor_follow.setIdleMode(IDLEMODE);
        intakeMotor.setOpenLoopRampRate(RAMPING_FROM_0_TO_FULL);
    }

    public void set(double speed) {
        intakeMotor.set(speed);
    }

    public Command runIntakeCommand(boolean reverse) {
        return runIntakeCommand(reverse ? -DEFAULT_SPEED : DEFAULT_SPEED);
    }

    public Command runIntakeCommand(double speed) {
        return new StartEndCommand(() -> this.set(speed), () -> this.set(0), this);
    }
}
