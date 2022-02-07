
 package frc.robot.subsystems;

import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.team5431.titan.core.misc.Calc;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

/**
 * @author Colin Wong
 * @author Rishmita Rao
 * @author Daniel Brubaker
 * @author Ryan Hirasaki
 */
public class Pivot extends SubsystemBase {
    public static enum POSITION {
        UP(Constants.PIVOT_UP_LIMIT), DOWN(Constants.PIVOT_DOWN_LIMIT), ZERO(0);

        private final double value;

        private POSITION(double value) {
            this.value = value;
        }

        public double getValue() {
            return value;
        }
    }

    private WPI_TalonFX pivotMotor;
    private POSITION position = POSITION.UP;

	private PIDController pidController;

    private NeutralMode nm;

    public Pivot(WPI_TalonFX pivotMotor) {
        this.pivotMotor = pivotMotor;
        this.pivotMotor.setInverted(Constants.PIVOT_REVERSE);
        setNeutralMode(Constants.PIVOT_NEUTRALMODE);
        this.pivotMotor.configFactoryDefault();
        
        // reset encoder
        this.pivotMotor.setSelectedSensorPosition(0);
        this.pivotMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0,
                Constants.DRIVEBASE_TIMEOUT_MS);

        // flywheel.setSensorPhase(true);

        this.pivotMotor.configPeakOutputForward(Constants.PIVOT_DEFAULT_SPEED);
        this.pivotMotor.configPeakOutputReverse(-Constants.PIVOT_DEFAULT_SPEED);


        this.pivotMotor.config_kF(0, Constants.PIVOT_MOTION_MAGIC.kF, Constants.DRIVEBASE_TIMEOUT_MS);
        this.pivotMotor.config_kP(0, Constants.PIVOT_MOTION_MAGIC.kP, Constants.DRIVEBASE_TIMEOUT_MS);
        this.pivotMotor.config_kI(0, Constants.PIVOT_MOTION_MAGIC.kI, Constants.DRIVEBASE_TIMEOUT_MS);
        this.pivotMotor.config_kD(0, Constants.PIVOT_MOTION_MAGIC.kD, Constants.DRIVEBASE_TIMEOUT_MS);
    }

    // return the angle of the arm based on the current encoder value
    // public double getAngle() {
    //     int currentPosition = pivotMotor.getSelectedSensorPosition();

    //     // 100:1

    //     // divide the encoder position when arm is horizontal by the angle displacement
    //     // so if you moved the arm 30 degrees and read 1000 ticks, it would be 1000/30
    //     // ticks per degree
    //     // subtract horizontalAngleDisplacement to make the horizontal postion 0 degrees
    //     // double ticksPerDegree = horizontalPosition / horizontalAngleDisplacement;
    //     double angle = (currentPosition * (2048 / 360)) / 100;
    //     return angle;
    // }

    @Override
    public void periodic() {
        // Publish Pivot Encoder Position To Add To States
        SmartDashboard.putNumber("Pivot Position", pivotMotor.getSelectedSensorPosition());

        // double angle = getAngle();
        // double theta = Math.toRadians(angle);

        // SmartDashboard.putNumber("Pivot Angle", angle);

        // // get a range of 0 to 1 to multiply by feedforward.
        // // when in horizontal position, value should be 1
        // // when in vertical up or down position, value should be 0
        // double gravityCompensation = Math.cos(theta);

        // SmartDashboard.putNumber("Pivot Gravity Compensation", gravityCompensation);

        // // horizontalHoldOutput is the minimum power required to hold the arm up when
        // // horizontal
        // // this is a range of 0-1, in our case it was .125 throttle required to keep the
        // // arm up
        // double feedForward = gravityCompensation * 0.125;

        // SmartDashboard.putNumber("Pivot Feed Forward", feedForward);

        SmartDashboard.putNumber("Pivot Sensor Velocity", pivotMotor.getSelectedSensorVelocity());
        SmartDashboard.putNumber("Pivot Speed", pivotMotor.get());
		SmartDashboard.putNumber("Pivot Error Rate", pivotMotor.getClosedLoopError(0));
        SmartDashboard.putString("Pivot Neutral Mode", nm.name());

        double horizontal = Constants.PIVOT_DOWN_LIMIT;
        double ticksToDegrees = (
                2048 // Encoder ticks per revolution
                / 360 // Degrees in one revolution
                ) * 100; // Effective gear ratio to account for degree turn
        double currentPosition = getEncoderPosition();
        double degrees = 
                Math.abs(currentPosition - horizontal) // Distance (in ticks) between current encoder value and "down" value
                / ticksToDegrees; // Convert ticks to degrees
        double radians = Math.toRadians(degrees); // Convert degrees to radians to prepare for cosine
        double CosineScalar = Math.cos(radians * Constants.PIVOT_COSINE_MULT); // Apply cosine to the degree measurement to get an inverse curve 
                                                 // When pivot is horizontal, the degree measurement is 0 but the scalar is 1
                                                 // When pivot is vertical, the degree measurement is about 60-90 but the scalar is very low (around 0)
        SmartDashboard.putNumber("Cosine Scalar", CosineScalar);
    }

    public void setPivotLocation(POSITION pos) {
        double horizontal = Constants.PIVOT_DOWN_LIMIT;
        double ticksToDegrees = (
                2048 // Encoder ticks per revolution
                / 360 // Degrees in one revolution
                ) * 100; // Effective gear ratio to account for degree turn
        double currentPosition = getEncoderPosition();
        double degrees = 
                Math.abs(currentPosition - horizontal) // Distance (in ticks) between current encoder value and "down" value
                / ticksToDegrees; // Convert ticks to degrees
        double radians = Math.toRadians(degrees); // Convert degrees to radians to prepare for cosine
        double CosineScalar = Math.cos(radians * Constants.PIVOT_COSINE_MULT); // Apply cosine to the degree measurement to get an inverse curve 
                                                 // When pivot is horizontal, the degree measurement is 0 but the scalar is 1
                                                 // When pivot is vertical, the degree measurement is about 60-90 but the scalar is very low (around 0)
        double maxGravity = Constants.PIVOT_AFFECT_GRAVITY;

        pivotMotor.set(ControlMode.Position, pos.getValue(), // Set position to parameter, using the built-in PID
                DemandType.ArbitraryFeedForward, CosineScalar * maxGravity); // Apply a constant power to the closed-loop output
                                                                             // Meaning that if the pivot is going from horizontal to vertical, the power applied will always be 1 (scalar) times maxGravity
                                                                             // If the pivot is going from vertical to horizontal, the power applied will always be close to 0
    }

    public void setNeutralMode(NeutralMode nm) {
        pivotMotor.setNeutralMode(nm);
        this.nm = nm;
    }
    
    private double getEncoderPosition() {
        return pivotMotor.getSelectedSensorPosition(0);
    }

    public boolean atLocation() {
        double encoderValue = pivotMotor.getSelectedSensorPosition();
        return Calc.approxEquals(position.getValue(), encoderValue, Constants.PIVOT_ERROR_RANGE);
    }

    public PIDController getPidController() {
        return pidController;
    }

	public void reset() {
        pivotMotor.setSelectedSensorPosition(0);
	}

	public double error() {
		return pivotMotor.getClosedLoopError();
    }
    
    public List<WPI_TalonFX> getMotors() {
        return List.of(pivotMotor);
    }
}