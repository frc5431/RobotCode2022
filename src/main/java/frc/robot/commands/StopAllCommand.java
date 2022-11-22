package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Systems;
import frc.robot.commands.subsystems.*;

public class StopAllCommand extends ParallelCommandGroup {
    public StopAllCommand(Systems systems) {
        addCommands(
            systems.getIntake().runIntakeCommand(0),
            systems.getFeeder().getBottom().runFeederCommand(0),
            systems.getFeeder().getTop().runFeederCommand(0),
            new ShooterCommand(systems, 0),
            new DriveCommand(systems, 0, 0, 0),
            new ClimberExtendCommand(systems, 0),
            new ClimberHingeCommand(systems, 0),
            systems.getPivot().runPivotCommand(0)
        );
    }
}
