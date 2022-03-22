package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Systems;
import frc.robot.commands.subsystems.*;
import frc.robot.subsystems.Shooter;

public class StopAllCommand extends ParallelCommandGroup {
    public StopAllCommand(Systems systems) {
        addCommands(
            new IntakeCommand(systems, 0),
            new FeederBottomCommand(systems, 0),
            new FeederTopCommand(systems, 0),
            new ShooterCommand(systems, Shooter.Velocity.OFF),
            new DriveCommand(systems, 0, 0, 0),
            new ClimberExtendCommand(systems, 0),
            new ClimberHingeCommand(systems, 0),
            new PivotCommand(systems, 0)
        );
    }
}
