package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Systems;
import frc.robot.commands.subsystems.FeederTopWaitCommand;

public class FloorIntakeCommand extends ParallelCommandGroup {
    public FloorIntakeCommand(Systems systems) {
        this(systems, true);
    }

    public FloorIntakeCommand(Systems systems, boolean useProxy) {
        Command bottom = systems.getFeeder().getBottom().runFeederCommand(false);
        Command top = new FeederTopWaitCommand(systems, false);

        addCommands(
            systems.getIntake().runIntakeCommand(false),
            useProxy ? bottom.asProxy() : bottom,
            useProxy ? top.asProxy() : top
        );
    }
}
