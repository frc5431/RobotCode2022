package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Systems;
import frc.robot.commands.subsystems.FeederCommand;
import frc.robot.commands.subsystems.IntakeCommand;

public class ShooterFeederIntakeCommand extends ParallelCommandGroup {
    public ShooterFeederIntakeCommand(Systems systems) {
        addCommands(
            new IntakeCommand(systems, true),
            new FeederCommand(systems, true),
            new ShootCommand(systems, true)
        );
    }
}
