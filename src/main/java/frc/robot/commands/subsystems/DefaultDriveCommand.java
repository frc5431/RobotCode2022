package frc.robot.commands.subsystems;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Systems;
import frc.robot.subsystems.Drivebase;
import frc.team5431.titan.core.leds.Blinkin;
import frc.team5431.titan.core.misc.Logger;

import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends CommandBase {
    private final Drivebase m_drivetrainSubsystem;
    private final Blinkin leds;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;

    public DefaultDriveCommand(Systems systems,
                               DoubleSupplier translationXSupplier,
                               DoubleSupplier translationYSupplier,
                               DoubleSupplier rotationSupplier) {
        this.m_drivetrainSubsystem = systems.getDrivebase();
        this.leds = systems.getLed();
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;

        addRequirements(m_drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        Logger.l("Default drive starting");
    }

    @Override
    public void execute() {
        double x = m_translationXSupplier.getAsDouble();
        double y = m_translationYSupplier.getAsDouble();
        double rot = m_rotationSupplier.getAsDouble();

        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        m_drivetrainSubsystem.driveController(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        x,
                        y,
                        rot,
                        m_drivetrainSubsystem.getGyroscopeRotation()
                )
        );

        if (Math.abs(x) > 0.1 || Math.abs(y) > 0.1 || Math.abs(rot) > 0.1) {
            leds.set(Constants.LEDPATTERN_DEFAULT);
        }
    }

    @Override
    public void end(boolean interrupted) {
        Logger.l("Default drive ending");
        m_drivetrainSubsystem.stop();
    }
}
