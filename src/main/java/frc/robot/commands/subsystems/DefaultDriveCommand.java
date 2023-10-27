package frc.robot.commands.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Systems;
import frc.robot.subsystems.Drivebase;
import frc.team5431.titan.core.leds.Blinkin;
import frc.team5431.titan.core.misc.Logger;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DefaultDriveCommand extends CommandBase {
    private final Drivebase m_drivetrainSubsystem;
    private final Blinkin leds;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;
    private final BooleanSupplier m_useFlickStick;
    private final Supplier<Pair<Double, Double>> m_stickSupplier;
    private PIDController rotController = null;


    public DefaultDriveCommand(Systems systems,
                               DoubleSupplier translationXSupplier,
                               DoubleSupplier translationYSupplier,
                               DoubleSupplier rotationSupplier,
                               BooleanSupplier useFlickStick,
                               Supplier<Pair<Double, Double>> stickSupplier) {
        this.m_drivetrainSubsystem = systems.getDrivebase();
        this.leds = systems.getLed();
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;
        this.m_useFlickStick = useFlickStick;
        this.m_stickSupplier = stickSupplier;

        addRequirements(m_drivetrainSubsystem);
    }

    public DefaultDriveCommand(Systems systems,
                               DoubleSupplier translationXSupplier,
                               DoubleSupplier translationYSupplier,
                               DoubleSupplier rotationSupplier,
                               BooleanSupplier useFlickStick) {
        this(
            systems,
            translationXSupplier,
            translationYSupplier,
            rotationSupplier,
            useFlickStick,
            () -> Pair.of(0.0, 0.0)
        );
    }

    @Override
    public void initialize() {
        Logger.l("Default drive starting");
        rotController = new PIDController(0.12, 0.12, 0.012);
        rotController.setTolerance(1, 2);
        rotController.enableContinuousInput(0, 360);
    }

    @Override
    public void execute() {
        double x = m_translationXSupplier.getAsDouble();
        double y = m_translationYSupplier.getAsDouble();
        double rot = m_rotationSupplier.getAsDouble();

        // Slow down robot if shooting while moving
        if (Drivebase.lockedToHub) {
            x /= 4;
            y /= 4;
        }

        if(m_useFlickStick.getAsBoolean()) {
            rotController.setSetpoint(rot);
            rot = rotController.calculate(m_drivetrainSubsystem.getGyroscopeRotation().getDegrees());
            var stick = m_stickSupplier.get();
            double speed = Math.sqrt(Math.pow(stick.getFirst(), 2) + Math.pow(stick.getSecond(), 2));

            rot = MathUtil.clamp(rot * speed, -Drivebase.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, Drivebase.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);
        }

        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        m_drivetrainSubsystem.driveController(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        x,
                        y,
                        rot,
                        m_drivetrainSubsystem.getGyroscopeRotation()
                )
        );



        // reset climb lights
        if (leds.getPattern() == Constants.LEDPATTERN_CLIMB &&
            (Math.abs(x) > 0.1 || Math.abs(y) > 0.1 || Math.abs(rot) > 0.1)) {
            leds.set(Constants.LEDPATTERN_DEFAULT);
        }
    }

    @Override
    public void end(boolean interrupted) {
        Logger.l("Default drive ending");
        m_drivetrainSubsystem.stop();
    }
}
