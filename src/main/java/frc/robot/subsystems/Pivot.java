
 package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.base.Calibratable;

import static java.lang.Math.max;
import static java.lang.Math.min;

/**
 * @author Colin Wong
 */
public class Pivot extends SubsystemBase implements Calibratable {
    public static final double PIVOT_DOWN_LIMIT = 1782074; //-42k // -45k // -49k
    public static final double PIVOT_UP_LIMIT = 0;
    public static final boolean REVERSE = false;
    public static final NeutralMode NEUTRALMODE = NeutralMode.Brake;
    public static final double DEFAULT_SPEED = 0.2;

    private WPI_TalonFX pivotMotor, _pivotFollow;

    public Pivot(WPI_TalonFX pivotMotorL, WPI_TalonFX pivotMotorR) {
        pivotMotor = pivotMotorL;
        _pivotFollow = pivotMotorR;

        pivotMotor.configFactoryDefault();
        _pivotFollow.configFactoryDefault();

        _pivotFollow.follow(pivotMotor);

        pivotMotor.setInverted(REVERSE);
		_pivotFollow.setInverted(InvertType.OpposeMaster); // Inverted via "!"
        // _pivotFollow.setInverted(!REVERSE);

        pivotMotor.setNeutralMode(NeutralMode.Brake);
        _pivotFollow.setNeutralMode(NeutralMode.Brake);

        // reset encoder
        pivotMotor.setSelectedSensorPosition(0);
        pivotMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);
        pivotMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        _pivotFollow.setSelectedSensorPosition(0);
        _pivotFollow.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);
        _pivotFollow.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);

        pivotMotor.configForwardSoftLimitEnable(!true);
        pivotMotor.configForwardSoftLimitThreshold(max(PIVOT_UP_LIMIT, PIVOT_DOWN_LIMIT));
        pivotMotor.configReverseSoftLimitEnable(!true);
        pivotMotor.configReverseSoftLimitThreshold(min(PIVOT_UP_LIMIT, PIVOT_DOWN_LIMIT));


        // this.pivotMotor.configPeakOutputForward(DEFAULT_SPEED);
        // this.pivotMotor.configPeakOutputReverse(-DEFAULT_SPEED);

        // this.pivotMotor.config_kP(0, PIVOT_kP);
        // this.pivotMotor.config_kI(0, PIVOT_kI);
        // this.pivotMotor.config_kD(0, PIVOT_kD);
        // this.pivotMotor.config_kF(0, PIVOT_kF);

        pivotMotor.set(0);
        // _pivotFollow.set(0);
        Constants.tab_subsystems.addNumber("Pivot Position", this.pivotMotor::getSelectedSensorPosition)
                .withPosition(18, 3)
                .withSize(2, 1);
    }

    public void setCalibrateMode(boolean value) {
        this.pivotMotor.configForwardSoftLimitEnable(!value);
        this.pivotMotor.configReverseSoftLimitEnable(!value);
    }

    public void resetEncoder() {
        this.pivotMotor.setSelectedSensorPosition(0);
    }

    public void set(double speed) {
        pivotMotor.set(speed);
        // _pivotFollow.set(speed);
    }

    public Command runPivotCommand(boolean reverse) {
        return runPivotCommand(reverse ? -DEFAULT_SPEED : DEFAULT_SPEED);
    }

    public Command runPivotCommand(double speed) {
        return new StartEndCommand(() -> this.set(speed), () -> this.set(0), this);
    }
}