package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Servo;
import frc.robot.subsystems.*;

public class Systems {
    private WPI_TalonFX feederBottom;
    private WPI_TalonFX feederTop;
    private WPI_TalonFX shooterLeft;
    private WPI_TalonFX shooterRight;
    private WPI_TalonFX intakeMotor;
    private WPI_TalonFX pivotMotor;
    private WPI_TalonFX climberExtend;
    private WPI_TalonFX climberHinge;

    private Servo anglerServo;

    private Feeder feeder;
    private Shooter shooter;
    private Intake intake;
    private Pivot pivot;
    private Climber climber;
    private Angler angler;

    public Systems() {
        feederBottom = new WPI_TalonFX(Constants.ID_FEEDER_BOTTOM);
        feederTop = new WPI_TalonFX(Constants.ID_FEEDER_TOP);
        shooterLeft = new WPI_TalonFX(Constants.ID_SHOOTER_LEFT);
        shooterRight = new WPI_TalonFX(Constants.ID_SHOOTER_RIGHT);
        intakeMotor = new WPI_TalonFX(Constants.ID_INTAKE);
        pivotMotor = new WPI_TalonFX(Constants.ID_PIVOT);
        climberExtend = new WPI_TalonFX(Constants.ID_CLIMBER_EXTEND);
        climberHinge = new WPI_TalonFX(Constants.ID_CLIMBER_HINGE);

        anglerServo = new Servo(Constants.ID_ANGLER);

        feeder = new Feeder(feederBottom,feederTop);
        shooter = new Shooter(shooterLeft, shooterRight);
        intake = new Intake(intakeMotor);
        pivot = new Pivot(pivotMotor);
        climber = new Climber(climberExtend, climberHinge);
        angler = new Angler(anglerServo);
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

    public Angler getAngler() {
        return angler;
    }
}
