package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {

    public static final double DEFAULT_SPEED_BOTTOM = 1.0;  // 0.9 // 0.7 // 0.6
    public static final double DEFAULT_SPEED_TOP = 1.0; // 1.0 // 0.8
    public static final NeutralMode NEUTRALMODE = NeutralMode.Brake;
    public static final boolean REVERSE_BOTTOM = false;
    public static final boolean REVERSE_TOP = true;

    private FeederBase feederBottom, feederTop;

    public Feeder(WPI_TalonFX motorBottom, WPI_TalonFX motorTop) {
        motorBottom.setInverted(REVERSE_BOTTOM);
        motorTop.setInverted(REVERSE_TOP);

        motorBottom.setNeutralMode(NEUTRALMODE);
        motorTop.setNeutralMode(NEUTRALMODE);

        feederBottom = new Bottom(motorBottom);
        feederTop = new Top(motorTop);
    }

    public void setBottom(double speed) {
        feederBottom.set(speed);
    }

    public void setTop(double speed) {
        feederTop.set(speed);
    }

    public FeederBase getBottom() {
        return feederBottom;
    }

    public FeederBase getTop() {
        return feederTop;
    }

    public Command feedEverythingCommand(boolean reverse) {
        return getBottom().runFeederCommand(reverse).alongWith(
               getTop().runFeederCommand(reverse));
    }

    public class FeederBase extends SubsystemBase {
        protected WPI_TalonFX motor;
        private final double DEFAULT_SPEED;

        public FeederBase(WPI_TalonFX motor, double defaultSpeed) {
            this.motor = motor;
            this.DEFAULT_SPEED = defaultSpeed;
        }

        public void set(double speed) {
            motor.set(ControlMode.PercentOutput, speed);
        }

        public Command runFeederCommand(boolean reverse) {
            return runFeederCommand(reverse ? -DEFAULT_SPEED : DEFAULT_SPEED);
        }

        public Command runFeederCommand(double speed) {
            return new StartEndCommand(() -> this.set(speed), () -> this.set(0), this);
        }
    }

    public class Bottom extends FeederBase {
        public Bottom(WPI_TalonFX motor) {
            super(motor, DEFAULT_SPEED_BOTTOM);
        }
    }

    public class Top extends FeederBase {
        public Top(WPI_TalonFX motor) {
            super(motor, DEFAULT_SPEED_TOP);
        }
    }
}