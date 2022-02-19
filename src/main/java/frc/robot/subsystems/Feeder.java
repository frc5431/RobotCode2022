package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {

    public static final double DEFAULT_SPEED = 0.2;
    public static final NeutralMode NEUTRALMODE = NeutralMode.Brake;
    public static final boolean REVERSE_BOTTOM = true;
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

    public class FeederBase extends SubsystemBase {
        protected WPI_TalonFX motor;

        public FeederBase(WPI_TalonFX motor) {
            this.motor = motor;
        }

        public void set(double speed) {
            motor.set(ControlMode.PercentOutput, speed);
        }
    }

    public class Bottom extends FeederBase {
        public Bottom(WPI_TalonFX motor) {
            super(motor);
        }
    }

    public class Top extends FeederBase {
        public Top(WPI_TalonFX motor) {
            super(motor);
        }
    }
}