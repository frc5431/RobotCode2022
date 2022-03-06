package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Systems;
import frc.robot.commands.subsystems.DriveCommand;
import frc.robot.commands.subsystems.PivotCommand;
import frc.robot.subsystems.Shooter;

public class AutonCommand extends SequentialCommandGroup {
    private static final double SHOOT_TIME = 5;
    private static final double DRIVE_TIME = 1.5;
    private static final double PIVOT_TIME = 2;

    public static enum State {
        NOTHING, SHOOT, SHOOT_DRIVE;
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
            default:
                break;
        }
    }
}
