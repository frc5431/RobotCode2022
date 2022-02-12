package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {

    private WPI_TalonFX feederBottom, feederTop; 

    public Feeder(WPI_TalonFX feederBottom, WPI_TalonFX feederTop) {
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