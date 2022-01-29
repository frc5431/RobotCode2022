package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;

public class Systems {
    private WPI_TalonSRX feeder_motor;
    private WPI_TalonSRX shooter_motor;
    private WPI_TalonFX intake_motor;
    private WPI_TalonSRX pivot_motor;
    private WPI_TalonFX le1Fx;
    private WPI_TalonFX le2Fx;
    private WPI_TalonFX re1Fx;
    private WPI_TalonFX re2Fx;

    private Feeder feeder;
    private Shooter shooter;
    private Intake intake;
    private Pivot pivot;
    private Climber climber;

    public Systems() {
        feeder = new Feeder(feeder_motor);
        shooter = new Shooter(shooter_motor);
        intake = new Intake(intake_motor);
        pivot = new Pivot(pivot_motor);
        climber = new Climber(le1Fx, re1Fx, re2Fx, le2Fx);
    }

    public Feeder getFeeder() {
        return feeder;
    }

    public Shooter getShooter() {
        return shooter;
    }

    public Intake getIntake() {
        return intake;
    }

    public Pivot getPivot() {
        return pivot;
    }

    public Climber getClimber() {
        return climber;
    }
}
