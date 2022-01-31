package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    private WPI_TalonSRX shooter1, shooter2;

    public Shooter(WPI_TalonSRX shooter1, WPI_TalonSRX shooter2) {
        this.shooter1 = shooter1;
        this.shooter1.setInverted(true);
       
        this.shooter2 = shooter2;
        this.shooter2.setInverted(true);
    }

    public void set(double speed) {
        shooter1.set(ControlMode.PercentOutput, speed);
        shooter2.set(ControlMode.PercentOutput, speed);

    }

    public boolean atVelocity() {
        return false;
    }
}