package frc.robot.commands.subsystems;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivebase;

public class DriveCommand extends CommandBase {
    private final Drivebase drivebase;
    private final double x;
    private final double y;
    private final double theta;

    public DriveCommand(Drivebase systems, double x, double y, double theta) {
        this.drivebase = systems;
        
        this.x = x;
        this.y = y;
        this.theta = theta;
    }

    @Override
    public void execute() {
        drivebase.driveRaw(new ChassisSpeeds(x, y, theta));
    }

    @Override
    public void end(boolean interrupted) {
        drivebase.stop();
    }
}
