package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Systems;
import frc.robot.subsystems.Drivebase;

public class PathCommand extends SequentialCommandGroup {
    public static double DRIVEBASE_SPEED_MULT = 0.9;

    public PathCommand(Systems systems, String path) {
        PathPlannerTrajectory trajectory = PathPlanner.loadPath(path, Drivebase.MAX_VELOCITY_METERS_PER_SECOND, 2.0);
        Drivebase drivebase = systems.getDrivebase();

        addCommands(
            new PPSwerveControllerCommand(
                    trajectory, 
                    () -> drivebase.m_odometry.getPoseMeters(), 
                    drivebase.m_kinematics, 
                    new PIDController(0.2, 0, 0), 
                    new PIDController(0.2, 0, 0), 
                    new ProfiledPIDController(1.5, 0, 0, new Constraints(Drivebase.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 3.14)),
                    (states) -> drivebase.driveRaw(multiply(drivebase.m_kinematics.toChassisSpeeds(states), DRIVEBASE_SPEED_MULT)), 
                    drivebase),
            new InstantCommand(
                    () -> drivebase.stop()
                    , drivebase)
        );
    }

    private static ChassisSpeeds multiply(ChassisSpeeds speeds, double mult) {
        speeds.vxMetersPerSecond *= mult;
        speeds.vyMetersPerSecond *= mult;
        return speeds;
    }
}
