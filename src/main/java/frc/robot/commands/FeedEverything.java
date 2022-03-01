package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Systems;
import frc.robot.commands.subsystems.FeederBottomCommand;
import frc.robot.commands.subsystems.ClimberHingeCommand;

public class FeedEverything extends ParallelCommandGroup {
    public FeedEverything(Systems systems, boolean reverse) {
        addCommands(
            new FeederBottomCommand(systems, reverse),
            new ClimberHingeCommand(systems, reverse)
        );
    }
}
