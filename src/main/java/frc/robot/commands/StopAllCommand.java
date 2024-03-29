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
            systems.getShooter().runShooterCommand(0),
            new DriveCommand(systems, 0, 0, 0),
            systems.getClimber().getExtend().runClimberCommand(0),
            systems.getClimber().getHinge().runClimberCommand(0),
            systems.getPivot().runPivotCommand(0)
        );
    }
}
