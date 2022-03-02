package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * @author Aahana Shrivastava
 * @author Colin Wong
 */
public class Climber extends SubsystemBase {

    public static final double DEFAULT_SPEED_EXTEND = 0.5;
    public static final double DEFAULT_SPEED_HINGE = 1;
    public static final NeutralMode NEUTRAL_MODE_EXTEND = NeutralMode.Brake;
    public static final NeutralMode NEUTRAL_MODE_HINGE = NeutralMode.Brake;
    public static final boolean REVERSE_EXTEND = false;
    public static final boolean REVERSE_HINGE = false;

    private ClimberBase climberExtend, climberHinge;

    public Climber(WPI_TalonFX motorExtend, WPI_TalonFX motorHinge) {
        motorExtend.setInverted(REVERSE_EXTEND);
        motorExtend.setNeutralMode(NEUTRAL_MODE_EXTEND);

        motorHinge.setInverted(REVERSE_HINGE);
        motorHinge.setNeutralMode(NEUTRAL_MODE_HINGE);

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
