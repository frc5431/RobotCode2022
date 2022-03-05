package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static java.lang.Math.max;
import static java.lang.Math.min;

/**
 * @author Aahana Shrivastava
 * @author Colin Wong
 */
public class Climber extends SubsystemBase {

    public static final double DEFAULT_SPEED_EXTEND = 1.0; // 0.7
    public static final double DEFAULT_SPEED_HINGE = 1;
    public static final NeutralMode NEUTRAL_MODE_EXTEND = NeutralMode.Brake;
    public static final NeutralMode NEUTRAL_MODE_HINGE = NeutralMode.Brake;
    public static final boolean REVERSE_EXTEND = false;
    public static final boolean REVERSE_HINGE = false;
    public static final double EXTEND_DOWN_LIMIT = 0;
    public static final double EXTEND_UP_LIMIT = -384000;
    public static final double HINGE_CLOSED_LIMIT = -80000;
    public static final double HINGE_OPEN_LIMIT = 1580000;

    private ClimberBase climberExtend, climberHinge;

    public Climber(WPI_TalonFX motorExtend, WPI_TalonFX motorHinge) {
        motorExtend.setInverted(REVERSE_EXTEND);
        motorExtend.setNeutralMode(NEUTRAL_MODE_EXTEND);
        motorHinge.setInverted(REVERSE_HINGE);
        motorHinge.setNeutralMode(NEUTRAL_MODE_HINGE);

        motorExtend.configForwardSoftLimitEnable(true);
        motorHinge.configForwardSoftLimitEnable(true);
        motorExtend.configReverseSoftLimitEnable(true);
        motorHinge.configReverseSoftLimitEnable(false); // TODO

        motorExtend.configForwardSoftLimitThreshold(max(EXTEND_DOWN_LIMIT, EXTEND_UP_LIMIT));
        motorHinge.configForwardSoftLimitThreshold(max(HINGE_CLOSED_LIMIT, HINGE_OPEN_LIMIT));
        motorExtend.configReverseSoftLimitThreshold(min(EXTEND_DOWN_LIMIT, EXTEND_UP_LIMIT));
        motorHinge.configReverseSoftLimitThreshold(min(HINGE_CLOSED_LIMIT, HINGE_OPEN_LIMIT));

        climberExtend = new ClimberExtend(motorExtend);
        climberHinge = new ClimberHinge(motorHinge);
    }

    public void setExtend(double speed) {
        climberExtend.set(speed);
    }

    public void setHinge(double speed) {
        climberHinge.set(speed);
    }

    public ClimberBase getExtend() {
        return climberExtend;
    }

    public ClimberBase getHinge() {
        return climberHinge;
    }

    public class ClimberBase extends SubsystemBase {
        protected WPI_TalonFX motor;

        public ClimberBase(WPI_TalonFX motor) {
            this.motor = motor;
        }

        public void set(double speed) {
            motor.set(speed);
        }

        public void calibrateMode(boolean value) {
            motor.configForwardSoftLimitEnable(!value);
            motor.configReverseSoftLimitEnable(!value);
        }
    
        public void reset() {
            motor.setSelectedSensorPosition(0);
        }
    }

    public class ClimberExtend extends ClimberBase {
        public ClimberExtend(WPI_TalonFX motor) {
            super(motor);

            Constants.tab_subsystems.addNumber("Climber Extend Position", () -> motor.getSelectedSensorPosition());
        }
    }

    public class ClimberHinge extends ClimberBase {
        public ClimberHinge(WPI_TalonFX motor) {
            super(motor);

            Constants.tab_subsystems.addNumber("Climber Hinge Position", () -> motor.getSelectedSensorPosition());
        }
    }
}
