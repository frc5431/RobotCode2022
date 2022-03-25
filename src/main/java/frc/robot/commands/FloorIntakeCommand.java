package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ProxyScheduleCommand;
import frc.robot.Systems;
import frc.robot.commands.subsystems.FeederBottomCommand;
import frc.robot.commands.subsystems.IntakeCommand;

public class FloorIntakeCommand extends ParallelCommandGroup {
    public FloorIntakeCommand(Systems systems) {
        this(systems, true);
    }

    public FloorIntakeCommand(Systems systems, boolean useProxy) {
        addCommands(
            new IntakeCommand(systems, false),
            useProxy ? new ProxyScheduleCommand(new FeederBottomCommand(systems, false)) : new FeederBottomCommand(systems, false)
            // useProxy ? new ProxyScheduleCommand(new FeederTopCommand(systems, false)) : new FeederTopCommand(systems, false)
        );
    }
}
