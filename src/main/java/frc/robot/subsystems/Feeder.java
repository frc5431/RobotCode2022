package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {

    private WPI_TalonSRX feederBottom, feederTop; 

    public Feeder(WPI_TalonSRX feederBottom, WPI_TalonSRX feederTop) {
        this.feederBottom = feederBottom;
        this.feederBottom.setInverted(true);

        this.feederTop = feederTop;
        this.feederTop.setInverted(true);
    }

    public void set(double speed) {
        feederBottom.set(ControlMode.PercentOutput, speed);
        feederTop.set(ControlMode.PercentOutput, speed);
    }
}