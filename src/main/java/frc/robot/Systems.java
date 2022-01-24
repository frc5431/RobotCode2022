package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class Systems {
    private WPI_TalonFX feeder;
    private WPI_TalonFX shooter;
    private WPI_TalonFX intake;
    private WPI_TalonSRX pivot;
    private WPI_TalonSRX climber;

    public Systems() {
        feeder = new WPI_TalonFX(0);
        shooter = new WPI_TalonFX(1);
        intake = new WPI_TalonFX(2);
        pivot = new WPI_TalonSRX(0);
        climber = new WPI_TalonSRX(0);
    }

    public WPI_TalonFX getFeeder() {
        return feeder;
    }

    public WPI_TalonFX getShooter() {
        return shooter;
    }

    public WPI_TalonFX getIntake() {
        return intake;
    }

    public WPI_TalonSRX getPivot() {
        return pivot;
    }

    public WPI_TalonSRX getClimber() {
        return climber;
    }
}
