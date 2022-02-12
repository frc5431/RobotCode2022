package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;

public class Systems {
    private WPI_TalonFX feederBottom;
    private WPI_TalonFX feederTop;
    private WPI_TalonFX shooterLeft;
    private WPI_TalonFX shooterRight;
    private WPI_TalonFX intake_motor;
    private WPI_TalonFX pivot_motor;
    private WPI_TalonFX left;
    private WPI_TalonFX right;

    private Feeder feeder;
    private Shooter shooter;
    private Intake intake;
    private Pivot pivot;
    private Climber climber;

    public Systems() {
        feeder = new Feeder(feederBottom,feederTop);
        shooter = new Shooter(shooterLeft, shooterRight);
        intake = new Intake(intake_motor);
        pivot = new Pivot(pivot_motor);
        climber = new Climber(left, right);
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
