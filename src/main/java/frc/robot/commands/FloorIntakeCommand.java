package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Systems;
import frc.robot.commands.subsystems.FeederBottomCommand;
import frc.robot.commands.subsystems.IntakeCommand;

public class FloorIntakeCommand extends ParallelCommandGroup {
    public FloorIntakeCommand(Systems systems) {
        addCommands(
            new IntakeCommand(systems, false),
            new FeederBottomCommand(systems, false)
        );
    }
}
