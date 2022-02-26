package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {

    public static final double DEFAULT_SPEED = 0.5;
    public static final NeutralMode NEUTRALMODE = NeutralMode.Brake;
    public static final boolean REVERSE_BOTTOM = true;
    public static final boolean REVERSE_TOP = true;

    private WPI_TalonFX feederBottom, feederTop;

    public Feeder(WPI_TalonFX motorBottom, WPI_TalonFX motorTop) {
        feederBottom = motorBottom;
        feederTop = motorTop;

        feederBottom.setInverted(REVERSE_BOTTOM);
        feederTop.setInverted(REVERSE_TOP);

        feederBottom.setNeutralMode(NEUTRALMODE);
        feederTop.setNeutralMode(NEUTRALMODE);
    }

    public void set(double speed) {
        feederBottom.set(speed);
        feederTop.set(speed);
    }

    public void setBottom(double speed) {
        feederBottom.set(speed);
    }

    public void setTop(double speed) {
        feederTop.set(speed);
    }
}