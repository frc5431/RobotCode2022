package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.subsystems.*;
import frc.team5431.titan.core.leds.Blinkin;

import static frc.robot.Constants.CANBUS_SUBSYSTEM;

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

    private DigitalInput upperFeederSensor;

    private Drivebase drivebase;

    private Feeder feeder;
    private Shooter shooter;
    private Intake intake;
    private Pivot pivot;
    private Climber climber;
    private Angler angler;

    private PhotonCamera camera;

    private Blinkin led;

    public Systems() {
        feederBottom = new WPI_TalonFX(Constants.ID_FEEDER_BOTTOM, CANBUS_SUBSYSTEM);
        feederTop = new WPI_TalonFX(Constants.ID_FEEDER_TOP, CANBUS_SUBSYSTEM);
        shooterLeft = new WPI_TalonFX(Constants.ID_SHOOTER_LEFT, CANBUS_SUBSYSTEM);
        shooterRight = new WPI_TalonFX(Constants.ID_SHOOTER_RIGHT, CANBUS_SUBSYSTEM);
        intakeMotor = new WPI_TalonFX(Constants.ID_INTAKE, CANBUS_SUBSYSTEM);
        pivotMotor = new WPI_TalonFX(Constants.ID_PIVOT, CANBUS_SUBSYSTEM);
        climberExtend = new WPI_TalonFX(Constants.ID_CLIMBER_EXTEND, CANBUS_SUBSYSTEM);
        climberHinge = new WPI_TalonFX(Constants.ID_CLIMBER_HINGE, CANBUS_SUBSYSTEM);

        anglerServo = new Servo(Constants.SLOT_ANGLER);

        upperFeederSensor = new DigitalInput(Constants.SLOT_FEEDER_SENSOR);

        feeder = new Feeder(feederBottom,feederTop);
        shooter = new Shooter(shooterLeft, shooterRight);
        intake = new Intake(intakeMotor);
        pivot = new Pivot(pivotMotor);
        climber = new Climber(climberExtend, climberHinge);
        angler = new Angler(anglerServo);

        camera = new PhotonCamera(Constants.CAMERA_NAME);

        led = new Blinkin(Constants.SLOT_LEDS);

        drivebase = new Drivebase(camera);
    }

    public Drivebase getDrivebase() {
        return drivebase;
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

    public DigitalInput getUpperFeederSensor() {
        return upperFeederSensor;
    }

    public PhotonCamera getCamera() {
        return camera;
    }

    public Blinkin getLed() {
        return led;
    }
}
