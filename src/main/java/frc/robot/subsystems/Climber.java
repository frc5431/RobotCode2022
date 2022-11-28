package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.base.Calibratable;
import frc.team5431.titan.core.leds.Blinkin;

import static java.lang.Math.max;
import static java.lang.Math.min;

import java.util.function.DoubleSupplier;

/**
 * @author Aahana Shrivastava
 * @author Colin Wong
 */
public class Climber extends SubsystemBase {

    public static final double DEFAULT_SPEED_EXTEND = 1.0; // 0.7
    public static final double DEFAULT_SPEED_HINGE = 1;
    public static final NeutralMode NEUTRAL_MODE_EXTEND = NeutralMode.Brake;
    public static final NeutralMode NEUTRAL_MODE_HINGE = NeutralMode.Brake;
    public static final boolean REVERSE_EXTEND = true;
    public static final boolean REVERSE_HINGE = false;
    public static final double EXTEND_DOWN_LIMIT = 0;
    public static final double EXTEND_UP_LIMIT = 391000; // 405k // 420000 // 384000
    public static final double HINGE_CLOSED_LIMIT = 0;
    public static final double HINGE_OPEN_LIMIT = -275000; // TODO: test if inversion needed?

    private ClimberBase climberExtend, climberHinge;

    public Climber(WPI_TalonFX motorExtend, WPI_TalonFX motorHinge, Blinkin leds) {
        motorExtend.setInverted(REVERSE_EXTEND);
        motorExtend.setNeutralMode(NEUTRAL_MODE_EXTEND);
        motorHinge.setInverted(REVERSE_HINGE);
        motorHinge.setNeutralMode(NEUTRAL_MODE_HINGE);

        motorExtend.configForwardSoftLimitEnable(true);
        motorHinge.configForwardSoftLimitEnable(true);
        motorExtend.configReverseSoftLimitEnable(true);
        motorHinge.configReverseSoftLimitEnable(true);

        motorExtend.configForwardSoftLimitThreshold(max(EXTEND_DOWN_LIMIT, EXTEND_UP_LIMIT));
        motorHinge.configForwardSoftLimitThreshold(max(HINGE_CLOSED_LIMIT, HINGE_OPEN_LIMIT));
        motorExtend.configReverseSoftLimitThreshold(min(EXTEND_DOWN_LIMIT, EXTEND_UP_LIMIT));
        motorHinge.configReverseSoftLimitThreshold(min(HINGE_CLOSED_LIMIT, HINGE_OPEN_LIMIT));

        climberExtend = new ClimberExtend(motorExtend, leds);
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

    public class ClimberBase extends SubsystemBase implements Calibratable {
        protected WPI_TalonFX motor;
        private final double DEFAULT_SPEED;

        public ClimberBase(WPI_TalonFX motor, double defaultSpeed) {
            this.motor = motor;
            this.DEFAULT_SPEED = defaultSpeed;
        }

        public void set(double speed) {
            motor.set(speed);
        }

        public void setCalibrateMode(boolean value) {
            motor.configForwardSoftLimitEnable(!value);
            motor.configReverseSoftLimitEnable(!value);
        }
    
        public void resetEncoder() {
            motor.setSelectedSensorPosition(0);
        }

        /**
         * Run climber command
         * 
         * @param reverse whether the climb mechanism should go outwards/from the robot (false) or towards/into the robot (true)
         */
        public Command runClimberCommand(boolean reverse) {
            return runClimberCommand(reverse ? -DEFAULT_SPEED : DEFAULT_SPEED);
        }

        public Command runClimberCommand(double speed) {
            return runClimberCommand(() -> speed);
        }

        public Command runClimberCommand(DoubleSupplier supplier) {
            return Commands.runEnd(() -> this.set(supplier.getAsDouble()), () -> this.set(0), this);
        }
    }

    public class ClimberExtend extends ClimberBase {
        private final Blinkin leds;

        public ClimberExtend(WPI_TalonFX motor, Blinkin leds) {
            super(motor, DEFAULT_SPEED_EXTEND);

            this.leds = leds;

            Constants.tab_subsystems.addNumber("Climber Extend Position", () -> motor.getSelectedSensorPosition())
                    .withPosition(15, 0)
                    .withSize(3, 1);
        }

        @Override
        public Command runClimberCommand(DoubleSupplier supplier) {
            return Commands.runEnd(() -> {
                double value = supplier.getAsDouble();
                this.set(value);
                if (Math.abs(value) > 0.01) {
                    leds.set(Constants.LEDPATTERN_CLIMB);
                }
            }, () -> this.set(0), this);
        }
    }

    public class ClimberHinge extends ClimberBase {
        public ClimberHinge(WPI_TalonFX motor) {
            super(motor, DEFAULT_SPEED_HINGE);

            Constants.tab_subsystems.addNumber("Climber Hinge Position", () -> motor.getSelectedSensorPosition())
                    .withPosition(15, 1)
                    .withSize(3, 1);
        }
    }
}
