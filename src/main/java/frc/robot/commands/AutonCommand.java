package frc.robot.commands;

import com.pathplanner.lib.*;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Systems;
import frc.robot.commands.subsystems.DriveCommand;
import frc.robot.commands.subsystems.IntakeCommand;
import frc.robot.commands.subsystems.PivotCommand;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Shooter;

public class AutonCommand extends SequentialCommandGroup {
    private static final double SHOOT_TIME = 5;
    private static final double DRIVE_TIME = 1.75;
    private static final double PIVOT_TIME = 0.65;

    public static final String[] PATHS = new String[] {
        "1 Start To First Ball",
        "2 First to Second",
        "3 Third and Fourth Ball",
        "4 Fourth To Shoot"
    };

    public static enum State {
        NOTHING, SHOOT, SHOOT_DRIVE, PATH
    }

    public AutonCommand(Systems systems, State state) {
        addCommands(new InstantCommand(() -> {
            // reset odometry
            PathPlannerTrajectory trajectory = PathPlanner.loadPath("New Path", Drivebase.MAX_VELOCITY_METERS_PER_SECOND/4, 1.0);
            PathPlannerState initialState = trajectory.getInitialState();
            systems.getDrivebase().resetOdometry(new Pose2d(initialState.poseMeters.getTranslation(), initialState.holonomicRotation));
        }, systems.getDrivebase()));

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
                addCommands(
                    new PathCommand(systems, "New Path")
                );
                break;
                // addCommands(
                //     new WaitCommand(PIVOT_TIME)
                //         .deadlineWith(new PivotCommand(systems, true)),
                //     new ParallelCommandGroup(
                //         new PathCommand(systems, PATHS[0])
                //             .deadlineWith(new IntakeCommand(systems, false))
                //     ),
                //     new WaitCommand(3)
                //         .deadlineWith(new AimAndShootCommand(systems)),
                //     new ParallelCommandGroup(
                //         new PathCommand(systems, PATHS[1])
                //             .deadlineWith(new IntakeCommand(systems, false))
                //     ),
                //     new WaitCommand(2.5)
                //         .deadlineWith(new AimAndShootCommand(systems)),
                //     new ParallelCommandGroup(
                //         new PathCommand(systems, PATHS[2])
                //             .deadlineWith(new IntakeCommand(systems, false))
                //     ),
                //     new WaitCommand(2)
                //         .deadlineWith(new FloorIntakeCommand(systems)),
                //     new PathCommand(systems, PATHS[3]),
                //     new WaitCommand(3)
                //         .deadlineWith(new AimAndShootCommand(systems))
                // );
                // break;
        }
    }
}
