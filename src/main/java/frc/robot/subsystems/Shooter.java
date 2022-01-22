package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    private WPI_TalonSRX shooter;

    public Shooter(WPI_TalonSRX shooter) {
        this.shooter = shooter;
        this.shooter.setInverted(true);

    }

    public void set(double speed) {
        shooter.set(ControlMode.PercentOutput, speed);
    }
}