package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

    private WPI_TalonFX shooter, _shooterFollow;

    public Shooter(WPI_TalonFX left, WPI_TalonFX right) {
        shooter = left;
		_shooterFollow = right;

		_shooterFollow.follow(shooter);

		// Set Inverted Mode
		shooter.setInverted(Constants.SHOOTER_FLYWHEEL_REVERSE);
		_shooterFollow.setInverted(InvertType.OpposeMaster); // Inverted via "!"
    }

    public void set(double speed) {
        shooter.set(ControlMode.PercentOutput, speed);
        _shooterFollow
        .set(ControlMode.PercentOutput, speed);

    }

    public boolean atVelocity() {
        return false;
    }
}
//Remember that 1 shooter will go forward and 1 in reverse