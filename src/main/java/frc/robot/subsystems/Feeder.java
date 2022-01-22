package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {

    private WPI_TalonSRX feeder;

    public Feeder(WPI_TalonSRX feeder) {
        this.feeder = feeder;
        this.feeder.setInverted(true);

    }

    public void set(double speed) {
        feeder.set(ControlMode.PercentOutput, speed);
    }
}