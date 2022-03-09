package frc.robot.commands;

import com.pathplanner.lib.*;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Systems;
import frc.robot.commands.subsystems.DriveCommand;
import frc.robot.commands.subsystems.PivotCommand;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Shooter;

public class AutonCommand extends SequentialCommandGroup {
    private static final double SHOOT_TIME = 5;
    private static final double DRIVE_TIME = 1.75;
    private static final double PIVOT_TIME = 2;

    public static enum State {
        NOTHING, SHOOT, SHOOT_DRIVE, PATH
    }

    public AutonCommand(Systems systems, State state) {
        switch (state) {
            case NOTHING:
                addCommands(
                    new WaitCommand(PIVOT_TIME)
                        .deadlineWith(new PivotCommand(systems, true))
                );
                break;
            case SHOOT:
                addCommands(
                    new WaitCommand(SHOOT_TIME)
                        .deadlineWith(new ShootCommand(systems, Shooter.Velocity.NORMAL)),
                    new WaitCommand(PIVOT_TIME)
                        .deadlineWith(new PivotCommand(systems, true))
                );
                break;
            case SHOOT_DRIVE:
                addCommands(
                    new WaitCommand(SHOOT_TIME)
                        .deadlineWith(new ShootCommand(systems, Shooter.Velocity.NORMAL)),
                    new WaitCommand(DRIVE_TIME)
                        .deadlineWith(new DriveCommand(systems, 0.3, 0.0, false)),
                    new WaitCommand(PIVOT_TIME)
                        .deadlineWith(new PivotCommand(systems, true))
                );
                break;
            case PATH:
                PathPlannerTrajectory trajectory = PathPlanner.loadPath("1 Start To First Ball", Drivebase.MAX_VELOCITY_METERS_PER_SECOND, 2.0);
                Drivebase drivebase = systems.getDrivebase();

                addCommands(
                    new ParallelCommandGroup(
                        new PPSwerveControllerCommand(
                                trajectory, 
                                () -> drivebase.m_odometry.getPoseMeters(), 
                                drivebase.m_kinematics, 
                                new PIDController(0.2, 0, 0), 
                                new PIDController(0.2, 0, 0), 
                                new ProfiledPIDController(0.2, 0, 0, new Constraints(Drivebase.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 3.14)),
                                (states) -> drivebase.driveRaw(drivebase.m_kinematics.toChassisSpeeds(states)), 
                                drivebase)
                            .andThen(new InstantCommand(
                                    () -> drivebase.stop()
                                    , drivebase))
                    ),
                    new WaitCommand(PIVOT_TIME)
                        .deadlineWith(new PivotCommand(systems, true))
                );
                break;
        }
    }
}
