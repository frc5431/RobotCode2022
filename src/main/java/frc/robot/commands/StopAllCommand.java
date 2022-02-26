package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Systems;
import frc.robot.commands.subsystems.FeederCommand;
import frc.robot.commands.subsystems.IntakeCommand;
import frc.robot.commands.subsystems.ShooterCommand;
import frc.robot.subsystems.Shooter;

public class StopAllCommand extends ParallelCommandGroup {
    public StopAllCommand(Systems systems) {
        addCommands(
            new IntakeCommand(systems, 0),
            new FeederCommand(systems, 0),
            new ShooterCommand(systems, Shooter.Velocity.OFF)
        );
    }
}
