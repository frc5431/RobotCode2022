package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.EntryNotification;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.subsystems.ShooterCommand;

public class Shooter extends SubsystemBase {

    public static final double VELOCITY_CLOSE = 10000;
    public static final double VELOCITY_FAR = 20000;
    public static final double VELOCITY_BUFFER = 300;

    public static final double MAX_VELOCITY = 21800;

    public static final NeutralMode NEUTRALMODE = NeutralMode.Coast;
    public static final boolean REVERSE = true;
    public static final double RAMPING_FROM_0_TO_FULL = 0.3;

    private static final double DEFAULT_KP = 0;
    private static final double DEFAULT_KI = 0;
    private static final double DEFAULT_KD = 0;
    private static final double DEFAULT_KF = 0.055;

    public static NetworkTableEntry entryKP = null;
    public static NetworkTableEntry entryKI = null;
    public static NetworkTableEntry entryKD = null;
    public static NetworkTableEntry entryKF = null;

    public static enum Velocity {
        OFF(0), CLOSE(VELOCITY_CLOSE), FAR(VELOCITY_FAR);

        private double velocity;

        Velocity(double velocity) {
            this.velocity = velocity;
        }

        public double getVelocity() {
            return velocity;
        }
    }

    private WPI_TalonFX shooter, _shooterFollow;

    public Shooter(WPI_TalonFX left, WPI_TalonFX right) {
        shooter = left;
		_shooterFollow = right;

		_shooterFollow.follow(shooter);

		// Set Inverted Mode
		shooter.setInverted(REVERSE);
		_shooterFollow.setInverted(InvertType.OpposeMaster); // Inverted via "!"

        shooter.setNeutralMode(NEUTRALMODE);
        _shooterFollow.setNeutralMode(NEUTRALMODE);

        shooter.configClosedloopRamp(RAMPING_FROM_0_TO_FULL);
        _shooterFollow.configClosedloopRamp(RAMPING_FROM_0_TO_FULL);
        shooter.configOpenloopRamp(RAMPING_FROM_0_TO_FULL);
        _shooterFollow.configOpenloopRamp(RAMPING_FROM_0_TO_FULL);

        shooter.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        _shooterFollow.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

        if (entryKP == null) {
            entryKP = Constants.tab_subsystems
                    .add("Shooter kP", DEFAULT_KP)
                    .withWidget(BuiltInWidgets.kNumberSlider)
                    .withProperties(Map.of(
                        "min", 0,
                        "max", 2,
                        "block increment", 0.01))
                    .getEntry();
        }
        if (entryKI == null) {
            entryKI = Constants.tab_subsystems
                    .add("Shooter kI", DEFAULT_KI)
                    .withWidget(BuiltInWidgets.kNumberSlider)
                    .withProperties(Map.of(
                        "min", 0,
                        "max", 2,
                        "block increment", 0.01))
                    .getEntry();
        }
        if (entryKD == null) {
            entryKD = Constants.tab_subsystems
                    .add("Shooter kD", DEFAULT_KD)
                    .withWidget(BuiltInWidgets.kNumberSlider)
                    .withProperties(Map.of(
                        "min", 0,
                        "max", 2,
                        "block increment", 0.01))
                    .getEntry();
        }
        if (entryKF == null) {
            entryKF = Constants.tab_subsystems
                    .add("Shooter kF", DEFAULT_KF)
                    .withWidget(BuiltInWidgets.kNumberSlider)
                    .withProperties(Map.of(
                        "min", 0,
                        "max", 0.5,
                        "block increment", 0.005))
                    .getEntry();
        }

        entryKP.addListener(this::updateP, 
                EntryListenerFlags.kNew | 
                EntryListenerFlags.kImmediate | 
                EntryListenerFlags.kUpdate);
        entryKI.addListener(this::updateI, 
                EntryListenerFlags.kNew | 
                EntryListenerFlags.kImmediate | 
                EntryListenerFlags.kUpdate);
        entryKD.addListener(this::updateD, 
                EntryListenerFlags.kNew | 
                EntryListenerFlags.kImmediate | 
                EntryListenerFlags.kUpdate);
        entryKF.addListener(this::updateF, 
                EntryListenerFlags.kNew | 
                EntryListenerFlags.kImmediate | 
                EntryListenerFlags.kUpdate);

        // shooter.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 50);

        shooter.set(ControlMode.Velocity, 0);

        Constants.tab_subsystems.addNumber("Shooter Supplier", () -> ShooterCommand.getSupplierToTab().getAsDouble());

        Constants.tab_subsystems.addBoolean("Shooter At Velocity", this::atVelocity);
        Constants.tab_subsystems.addNumber("Shooter Target", shooter::getClosedLoopTarget);
        Constants.tab_subsystems.addNumber("Shooter Velocity", shooter::getSelectedSensorVelocity);
        Constants.tab_subsystems.addNumber("Shooter get()", shooter::get);
    }

    public void updateP(EntryNotification notif) {
        shooter.config_kP(0, notif.value.getDouble());
        _shooterFollow.config_kP(0, notif.value.getDouble());
    }

    public void updateI(EntryNotification notif) {
        shooter.config_kI(0, notif.value.getDouble());
        _shooterFollow.config_kI(0, notif.value.getDouble());
    }

    public void updateD(EntryNotification notif) {
        shooter.config_kD(0, notif.value.getDouble());
        _shooterFollow.config_kD(0, notif.value.getDouble());
    }

    public void updateF(EntryNotification notif) {
        shooter.config_kF(0, notif.value.getDouble());
        _shooterFollow.config_kF(0, notif.value.getDouble());
    }

    public void set(Shooter.Velocity velocity) {
        set(velocity.getVelocity());
    }

    public void set(double velocity) {
        shooter.set(ControlMode.Velocity, velocity);
    }

    public boolean atVelocity() {
        return shooter.getClosedLoopError() < VELOCITY_BUFFER;
    }
}
//Remember that 1 shooter will go forward and 1 in reverse