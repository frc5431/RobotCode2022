package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    public static final double DEFAULT_SPEED = 1.0;
    public static final boolean REVERSE = false;
    public static final NeutralMode NEUTRALMODE = NeutralMode.Coast;
    public static final double RAMPING_FROM_0_TO_FULL = 0.0; // 0.5

    private WPI_VictorSPX intakeMotor;
    private WPI_VictorSPX intakeMotor_follow;

    public Intake(WPI_VictorSPX leftMotor, WPI_VictorSPX rightMotor) {
        intakeMotor = leftMotor;
        intakeMotor_follow = rightMotor;
        intakeMotor_follow.follow(intakeMotor);
        intakeMotor.setInverted(REVERSE);
        intakeMotor_follow.setInverted(InvertType.OpposeMaster);
        intakeMotor.setNeutralMode(NEUTRALMODE);
        intakeMotor.configOpenloopRamp(RAMPING_FROM_0_TO_FULL);
        // intakeMotor.setOpenLoopRampRate(RAMPING_FROM_0_TO_FULL);
    }

    public void set(double speed) {
        intakeMotor.set(ControlMode.PercentOutput, speed);
    }

    public Command runIntakeCommand(boolean reverse) {
        return runIntakeCommand(reverse ? -DEFAULT_SPEED : DEFAULT_SPEED);
    }

    public Command runIntakeCommand(double speed) {
        return new StartEndCommand(() -> this.set(speed), () -> this.set(0), this);
    }
}
