package frc.robot.commands;

import com.pathplanner.lib.*;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Systems;
import frc.robot.commands.subsystems.DriveCommand;
import frc.robot.commands.subsystems.FeederBottomCommand;
import frc.robot.commands.subsystems.FeederTopCommand;
import frc.robot.commands.subsystems.IntakeCommand;
import frc.robot.commands.subsystems.PivotCommand;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Shooter;

public class AutonCommand extends SequentialCommandGroup {
    private static final double SHOOT_TIME = 4;
    private static final double PIVOT_TIME = 0.45;

    public static final String[] PATHS = new String[] {
        "1 Start To First Ball",
        "2 First to Second",
        "3 Third and Fourth Ball",
        "4 Fourth To Shoot"
    };

    public static enum State {
        NO_BALL, ONE_BALL, TWO_BALL, THREE_BALL, FOUR_BALL, FIVE_BALL, SIX_BALL, TEST_PATH, JUST_PATH
    }

    public AutonCommand(Systems systems, State state) {
        switch (state) {
            case NO_BALL:
                addCommands();
                break;
            case ONE_BALL:
                addCommands(
                    new WaitCommand(SHOOT_TIME)
                        .deadlineWith(new ShootCommand(systems, Shooter.Velocity.NORMAL, false)),
                    new WaitCommand(PIVOT_TIME)
                        .deadlineWith(new PivotCommand(systems, true))
                );
                break;
            case TWO_BALL:
                addCommands(
                    // commandResetAuton(systems, PATHS[0]),
                    // new WaitCommand(PIVOT_TIME)
                    //     .deadlineWith(new PivotCommand(systems, true)),
                    // new ParallelCommandGroup(
                    //     new PathCommand(systems, PATHS[0])
                    //         .deadlineWith(new FloorIntakeCommand(systems, false))
                    // ),
                    // new WaitCommand(3)
                    //     .deadlineWith(new ShootPlusCommand(systems)
                    //             .alongWith(new IntakeCommand(systems, false)))
                    new InstantCommand(() -> systems.getDrivebase().resetGyroAt(0), systems.getDrivebase()), // 146
                    new WaitCommand(PIVOT_TIME)
                        .deadlineWith(new PivotCommand(systems, true)),
                    new WaitCommand(1.7)
                        .deadlineWith(new DriveCommand(systems, 0.3, 0.0, false)
                                .alongWith(new FloorIntakeCommand(systems, false))),
                    new WaitCommand(0.3)
                        .deadlineWith(new FeederBottomCommand(systems, true)),
                    new WaitCommand(0.5)
                        .deadlineWith(new FeederTopCommand(systems, true)),
                    new WaitCommand(SHOOT_TIME)
                            .deadlineWith(new AimAndShootCommand(systems, false)),
                    new InstantCommand(() -> systems.getDrivebase().resetGyroAt(146), systems.getDrivebase())
                );
                break;
            case THREE_BALL:
                addCommands(
                    commandResetAuton(systems, PATHS[0]),
                    new WaitCommand(PIVOT_TIME)
                        .deadlineWith(new PivotCommand(systems, true)),
                    new ParallelCommandGroup(
                        new PathCommand(systems, PATHS[0])
                            .deadlineWith(new FloorIntakeCommand(systems, false))
                    ),
                    new WaitCommand(3)
                        .deadlineWith(new ShootPlusCommand(systems, false)
                                .alongWith(new IntakeCommand(systems, false))),
                    new ParallelCommandGroup(
                        new PathCommand(systems, PATHS[1])
                            .deadlineWith(new FloorIntakeCommand(systems, false))
                    ),
                    new WaitCommand(2.5)
                        .deadlineWith(new ShootPlusCommand(systems, false)
                                .alongWith(new IntakeCommand(systems, false)))
                );
                break;
            case FOUR_BALL:
                addCommands(
                    new InstantCommand(() -> systems.getDrivebase().resetGyroAt(0), systems.getDrivebase()), // 146
                    new WaitCommand(PIVOT_TIME)
                        .deadlineWith(new PivotCommand(systems, true)),
                    new WaitCommand(1.7)
                        .deadlineWith(new DriveCommand(systems, 1.4, -0.2, 0.0)
                                .alongWith(new FloorIntakeCommand(systems, false))),
                    new WaitCommand(0.25)
                        .deadlineWith(new FeederBottomCommand(systems, true)),
                    new WaitCommand(0.4)
                        .deadlineWith(new FeederTopCommand(systems, true)),
                    new WaitCommand(2.85)
                            .deadlineWith(new AimAndShootCommand(systems, false)),
                    new WaitCommand(2.0)
                        .deadlineWith(new DriveCommand(systems, 1.5, -0.6, 0.0)
                                .alongWith(new FloorIntakeCommand(systems, false))),
                    new WaitCommand(1.6)
                        .deadlineWith(new FloorIntakeCommand(systems, false)),
                    new WaitCommand(2.0)
                        .deadlineWith(new DriveCommand(systems, -1.5, 0.5, 0.0)
                                .alongWith(new FloorIntakeCommand(systems, false))),
                    new InstantCommand(() -> systems.getDrivebase().resetGyroAt(-155.6), systems.getDrivebase()),
                    new WaitCommand(0.25)
                        .deadlineWith(new FeederBottomCommand(systems, true)),
                    new WaitCommand(0.4)
                        .deadlineWith(new FeederTopCommand(systems, true)),
                    new WaitCommand(2.85)
                            .deadlineWith(new AimAndShootCommand(systems, false))
                );
                break;
            case FIVE_BALL:
                addCommands(
                    commandResetAuton(systems, PATHS[0]),
                    new WaitCommand(PIVOT_TIME)
                        .deadlineWith(new PivotCommand(systems, true)),
                    new ParallelCommandGroup(
                        new PathCommand(systems, PATHS[0])
                            .deadlineWith(new FloorIntakeCommand(systems, false))
                    ),
                    new WaitCommand(3)
                        .deadlineWith(new AimAndShootCommand(systems, false)),
                    new ParallelCommandGroup(
                        new PathCommand(systems, PATHS[1])
                            .deadlineWith(new FloorIntakeCommand(systems, false))
                    ),
                    new WaitCommand(2.5)
                        .deadlineWith(new AimAndShootCommand(systems, false)),
                    new ParallelCommandGroup(
                        new PathCommand(systems, PATHS[2])
                            .deadlineWith(new FloorIntakeCommand(systems, false))
                    ),
                    new WaitCommand(2)
                        .deadlineWith(new FloorIntakeCommand(systems, false)),
                    new PathCommand(systems, PATHS[3]),
                    new AimAndShootCommand(systems, false)
                );
                break;
            case JUST_PATH:
                addCommands(
                    commandResetAuton(systems, PATHS[0]),
                    new PathCommand(systems, PATHS[0]),
                    // commandResetAuton(systems, PATHS[1]),
                    new PathCommand(systems, PATHS[1])
                    // commandResetAuton(systems, PATHS[2]),
                    // new PathCommand(systems, PATHS[2]),
                    // commandResetAuton(systems, PATHS[3]),
                    // new PathCommand(systems, PATHS[3])
                );
                break;
            case TEST_PATH:
                addCommands(
                    commandResetAuton(systems, "New Path"),
                    new PathCommand(systems, "New Path")
                );
                break;
            default:
                break;
        }
    }

    public static CommandBase commandResetAuton(Systems systems, String path) {
        return new InstantCommand(() -> {
            // reset odometry
            PathPlannerTrajectory trajectory = PathPlanner.loadPath(path, Drivebase.MAX_VELOCITY_METERS_PER_SECOND, 1.0);
            PathPlannerState initialState = trajectory.getInitialState();
            systems.getDrivebase().resetGyroAt(initialState.holonomicRotation.getDegrees());
            systems.getDrivebase().resetOdometry(new Pose2d(initialState.poseMeters.getTranslation(), initialState.holonomicRotation));
        }, systems.getDrivebase());
    }
}
