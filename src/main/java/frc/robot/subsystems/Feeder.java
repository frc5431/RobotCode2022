package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {

    private WPI_TalonSRX feeder1, feeder2; 

    public Feeder(WPI_TalonSRX feeder1, WPI_TalonSRX feeder2) {
        this.feeder1 = feeder1;
        this.feeder1.setInverted(true);

        this.feeder2 = feeder2;
        this.feeder2.setInverted(true);
    }

    public void set(double speed) {
        feeder1.set(ControlMode.PercentOutput, speed);
        feeder2.set(ControlMode.PercentOutput, speed);
    }
}