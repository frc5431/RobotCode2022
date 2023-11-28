package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX; 

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    public static final double DEFAULT_SPEED = 0.7; //0.5
    public static final boolean REVERSE = true;
    public static final NeutralMode IDLEMODE = NeutralMode.Coast;
    public static final double RAMPING_FROM_0_TO_FULL = 0.0;  // 0.5

    private WPI_TalonSRX intakeMotor, intakeMotor_follow;  

    public Intake(WPI_TalonSRX leftPivot, WPI_TalonSRX rightPivot) {  
        intakeMotor = leftPivot;
        intakeMotor_follow = rightPivot;
        intakeMotor_follow.follow(intakeMotor);
        intakeMotor.setInverted(REVERSE);
        intakeMotor.configPeakCurrentLimit(10);
        intakeMotor.configPeakCurrentLimit(10);
        intakeMotor.setNeutralMode(IDLEMODE);
        intakeMotor_follow.setNeutralMode(IDLEMODE);
        intakeMotor.configOpenloopRamp(RAMPING_FROM_0_TO_FULL);  
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
