package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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
    private WPI_TalonSRX intakeMotorLeft;
    private WPI_TalonSRX intakeMotorRight;
    private WPI_TalonFX pivotMotorLeft;
    private WPI_TalonFX pivotMotorRight;
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
    private Vision vision;

    private Blinkin led;

    public Systems() {
        feederBottom = new WPI_TalonFX(Constants.ID_FEEDER_BOTTOM, CANBUS_SUBSYSTEM);
        feederTop = new WPI_TalonFX(Constants.ID_FEEDER_TOP, CANBUS_SUBSYSTEM);
        shooterLeft = new WPI_TalonFX(Constants.ID_SHOOTER_LEFT, CANBUS_SUBSYSTEM);
        shooterRight = new WPI_TalonFX(Constants.ID_SHOOTER_RIGHT, CANBUS_SUBSYSTEM);
       
        intakeMotorLeft = new WPI_TalonSRX(Constants.ID_INTAKE_LEFT);
        intakeMotorRight = new WPI_TalonSRX(Constants.ID_INTAKE_RIGHT);
       
        pivotMotorLeft = new WPI_TalonFX(Constants.ID_PIVOT_LEFT, CANBUS_SUBSYSTEM);
        pivotMotorRight = new WPI_TalonFX(Constants.ID_PIVOT_RIGHT, CANBUS_SUBSYSTEM);
        climberExtend = new WPI_TalonFX(Constants.ID_CLIMBER_EXTEND, CANBUS_SUBSYSTEM);
        climberHinge = new WPI_TalonFX(Constants.ID_CLIMBER_HINGE, CANBUS_SUBSYSTEM);

        anglerServo = new Servo(Constants.SLOT_ANGLER);

        upperFeederSensor = new DigitalInput(Constants.SLOT_FEEDER_SENSOR);

        led = new Blinkin(Constants.SLOT_LEDS);

        feeder = new Feeder(feederBottom,feederTop);
        shooter = new Shooter(shooterLeft, shooterRight);
        intake = new Intake(intakeMotorLeft, intakeMotorRight);
        pivot = new Pivot(pivotMotorLeft, pivotMotorRight);
        climber = new Climber(climberExtend, climberHinge, led);
        angler = new Angler(anglerServo);

        camera = new PhotonCamera(Constants.CAMERA_NAME);

        drivebase = new Drivebase(camera);
        // vision = new Vision(drivebase);
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

    public Vision getVision() {
        return vision;
    }

    public Blinkin getLed() {
        return led;
    }
}
