
 package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static java.lang.Math.max;
import static java.lang.Math.min;

/**
 * @author Colin Wong
 * @author Rishmita Rao
 * @author Daniel Brubaker
 * @author Ryan Hirasaki
 */
public class Pivot extends SubsystemBase {
    public static final double PIVOT_DOWN_LIMIT = -45000; //-42k // -45k // -49k
    public static final double PIVOT_UP_LIMIT = 0;
    public static final boolean REVERSE = false;
    public static final NeutralMode NEUTRALMODE = NeutralMode.Brake;
    public static final double DEFAULT_SPEED = 0.5;
    private static final double PIVOT_kP = 0;
    private static final double PIVOT_kI = 0;
    private static final double PIVOT_kD = 0;
    private static final double PIVOT_kF = 1;

    private WPI_TalonFX pivotMotor;

    public Pivot(WPI_TalonFX pivotMotor) {
        this.pivotMotor = pivotMotor;
        this.pivotMotor.configFactoryDefault();
        this.pivotMotor.setInverted(REVERSE);
        this.pivotMotor.setNeutralMode(NEUTRALMODE);
        
        // reset encoder
        this.pivotMotor.setSelectedSensorPosition(0);
        this.pivotMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);
        this.pivotMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        
        this.pivotMotor.configForwardSoftLimitEnable(true);
        this.pivotMotor.configForwardSoftLimitThreshold(max(PIVOT_UP_LIMIT, PIVOT_DOWN_LIMIT));
        this.pivotMotor.configReverseSoftLimitEnable(true);
        this.pivotMotor.configReverseSoftLimitThreshold(min(PIVOT_UP_LIMIT, PIVOT_DOWN_LIMIT));

        // flywheel.setSensorPhase(true);

        this.pivotMotor.configPeakOutputForward(DEFAULT_SPEED);
        this.pivotMotor.configPeakOutputReverse(-DEFAULT_SPEED);

        this.pivotMotor.config_kP(0, PIVOT_kP);
        this.pivotMotor.config_kI(0, PIVOT_kI);
        this.pivotMotor.config_kD(0, PIVOT_kD);
        this.pivotMotor.config_kF(0, PIVOT_kF);

        Constants.tab_subsystems.addNumber("Pivot Position", this.pivotMotor::getSelectedSensorPosition)
                .withPosition(18, 3)
                .withSize(2, 1);
    }

    public void calibrateMode(boolean value) {
        this.pivotMotor.configForwardSoftLimitEnable(!value);
        this.pivotMotor.configReverseSoftLimitEnable(!value);
    }

    public void reset() {
        this.pivotMotor.setSelectedSensorPosition(0);
    }

    public void set(double speed) {
        pivotMotor.set(speed);
    }

    public Command runPivotCommand(boolean reverse) {
        return runPivotCommand(reverse ? -DEFAULT_SPEED : DEFAULT_SPEED);
    }

    public Command runPivotCommand(double speed) {
        return new StartEndCommand(() -> this.set(speed), () -> this.set(0), this);
    }
}