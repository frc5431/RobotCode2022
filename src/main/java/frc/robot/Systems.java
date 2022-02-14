package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class Systems {
    private WPI_TalonFX feederBottom;
    private WPI_TalonFX feederTop;
    private WPI_TalonFX shooterLeft;
    private WPI_TalonFX shooterRight;
    private WPI_TalonFX intakeMotor;

    private Feeder feeder;
    private Shooter shooter;
    private Intake intake;

    public Systems() {
        feederBottom = new WPI_TalonFX(Constants.ID_FEEDER_BOTTOM);
        feederTop = new WPI_TalonFX(Constants.ID_FEEDER_TOP);
        shooterLeft = new WPI_TalonFX(Constants.ID_SHOOTER_LEFT);
        shooterRight = new WPI_TalonFX(Constants.ID_SHOOTER_RIGHT);
        intakeMotor = new WPI_TalonFX(Constants.ID_INTAKE);

        feeder = new Feeder(feederBottom,feederTop);
        shooter = new Shooter(shooterLeft, shooterRight);
        intake = new Intake(intakeMotor);
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
}
