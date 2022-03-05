package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Systems;
import frc.robot.commands.subsystems.DriveCommand;
import frc.robot.subsystems.Shooter;

public class AutonCommand extends SequentialCommandGroup {
    public static enum State {
        NOTHING, SHOOT, SHOOT_DRIVE;
    }

    public AutonCommand(Systems systems, State state) {
        switch (state) {
            case NOTHING:
                break;
            case SHOOT:
                addCommands(
                    new WaitCommand(5)
                        .deadlineWith(new ShootCommand(systems, Shooter.Velocity.NORMAL))
                );
                break;
            case SHOOT_DRIVE:
                addCommands(
                    new WaitCommand(5)
                        .deadlineWith(new ShootCommand(systems, Shooter.Velocity.NORMAL)),
                    new WaitCommand(1.5)
                        .deadlineWith(new DriveCommand(systems, 0.3, 0.0, false))
                );
                break;
            default:
                break;
        }
    }
}
