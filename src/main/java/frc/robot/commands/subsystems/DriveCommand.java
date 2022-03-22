package frc.robot.commands.subsystems;

import org.apache.commons.lang3.tuple.Triple;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Systems;
import frc.robot.subsystems.Drivebase;
import frc.team5431.titan.core.misc.Logger;

public class DriveCommand extends CommandBase {
    private final Drivebase drivebase;

    private final Triple<Double, Double, Double> fieldRelative;
    private final Triple<Double, Double, Boolean> robotRelative;

    public DriveCommand(Systems systems, double x, double y, double theta) {
        this.drivebase = systems.getDrivebase();
        
        this.fieldRelative = Triple.of(x, y, theta);
        this.robotRelative = null;

        addRequirements(drivebase);
    }

    public DriveCommand(Systems systems, double drive, double turn, boolean curve) {
        this.drivebase = systems.getDrivebase();

        this.fieldRelative = null;
        this.robotRelative = Triple.of(drive, turn, curve);

        addRequirements(drivebase);
    }

    @Override
    public void initialize() {
        Logger.l("Drive Command Starting");
    }

    @Override
    public void execute() {
        if (fieldRelative != null) {
            Logger.l("Driving field-relative! %s", fieldRelative);
            drivebase.driveRaw(new ChassisSpeeds(fieldRelative.getLeft(), fieldRelative.getMiddle(), fieldRelative.getRight()));
        } else if (robotRelative != null) {
            Logger.l("Driving robot-relative! %s", robotRelative);
            drivebase.driveRelative(robotRelative.getLeft(), robotRelative.getMiddle(), robotRelative.getRight());
        } else {
            Logger.e("Drive Command has no valid values!");
        }
    }

    @Override
    public void end(boolean interrupted) {
        Logger.l("Drive Command Ending");
        drivebase.stop();
    }
}
