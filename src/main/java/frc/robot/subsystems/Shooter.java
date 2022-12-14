package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

    public static final double VELOCITY_REJECT = 6892;
    public static final double VELOCITY_NORMAL = 13000; // 13000
    public static final double VELOCITY_BUFFER = 100;

    public static final double MAX_VELOCITY = 21350; // theoretical 21800

    public static final NeutralMode NEUTRALMODE = NeutralMode.Coast;
    public static final boolean REVERSE = true;
    public static final double RAMPING_FROM_0_TO_FULL = 0.3;

    private static final double DEFAULT_KP = 0.12;
    private static final double DEFAULT_KI = 0;
    private static final double DEFAULT_KD = 0;
    private static final double DEFAULT_KF = 0.05; // 0.055

    private WPI_TalonFX shooter, _shooterFollow;

    public Shooter(WPI_TalonFX left, WPI_TalonFX right) {
        shooter = left;
		_shooterFollow = right;

		_shooterFollow.follow(shooter);

		// Set Inverted Mode
		shooter.setInverted(REVERSE);
		_shooterFollow.setInverted(InvertType.OpposeMaster); // Inverted via "!"

        shooter.setNeutralMode(NEUTRALMODE);
        _shooterFollow.setNeutralMode(NEUTRALMODE);

        shooter.configClosedloopRamp(RAMPING_FROM_0_TO_FULL);
        _shooterFollow.configClosedloopRamp(RAMPING_FROM_0_TO_FULL);
        shooter.configOpenloopRamp(RAMPING_FROM_0_TO_FULL);
        _shooterFollow.configOpenloopRamp(RAMPING_FROM_0_TO_FULL);

        shooter.config_kP(0, DEFAULT_KP);
        _shooterFollow.config_kP(0, DEFAULT_KP);
        shooter.config_kI(0, DEFAULT_KI);
        _shooterFollow.config_kI(0, DEFAULT_KI);
        shooter.config_kD(0, DEFAULT_KD);
        _shooterFollow.config_kD(0, DEFAULT_KD);
        shooter.config_kF(0, DEFAULT_KF);
        _shooterFollow.config_kF(0, DEFAULT_KF);

        shooter.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 50);
        _shooterFollow.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 50);

        shooter.set(ControlMode.Velocity, 0);
        Constants.tab_subsystems.addNumber("Shooter Target", shooter::getClosedLoopTarget)
                .withPosition(2, 0)
                .withSize(2, 1);
        Constants.tab_subsystems.addNumber("Shooter Velocity", shooter::getSelectedSensorVelocity)
                .withPosition(2, 1)
                .withSize(2, 1);
        Constants.tab_subsystems.addBoolean("Shooter At Velocity", this::atVelocity)
                .withPosition(0, 5)
                .withSize(2, 1);
    }

    public void set(double velocity) {
        shooter.set(ControlMode.Velocity, velocity);
    }

    public boolean atVelocity() {
        return shooter.getClosedLoopError() < VELOCITY_BUFFER;
    }

    public Command runShooterCommand(double velocity) {
        return runShooterCommand(() -> velocity);
    }

    public Command runShooterCommand(DoubleSupplier velocitySupplier) {
        return Commands.runEnd(() -> this.set(velocitySupplier.getAsDouble()), () -> this.set(0), this);
    }
}
//Remember that 1 shooter will go forward and 1 in reverse