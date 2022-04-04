package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Systems;
import frc.robot.commands.subsystems.IntakeCommand;

public class AimAndShootCommand extends ParallelCommandGroup {
    public static final double AIM_LENGTH = 0.75;

    /*
     * 7.5m  - 17750 - 0.65
     * 2.15m - 11240 - 0.45
     */

    public AimAndShootCommand(Systems systems) {
        addCommands(
            new WaitCommand(AIM_LENGTH)
                .deadlineWith(new AimCommand(systems, true)),
            new WaitCommand(AIM_LENGTH)
                .andThen(new ShootPlusCommand(systems)),
            new IntakeCommand(systems, false)
        );
    }
}
