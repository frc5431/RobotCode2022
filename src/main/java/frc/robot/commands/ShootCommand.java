package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Systems;
import frc.robot.commands.subsystems.FeederBottomCommand;
import frc.robot.commands.subsystems.FeederTopCommand;
import frc.robot.commands.subsystems.ShooterCommand;
import frc.robot.subsystems.Shooter;

public class ShootCommand extends ParallelCommandGroup {
    public static final double SHOOTER_WAIT_TILL_SPEED = 1;

    public ShootCommand(Systems systems, Shooter.Velocity velocity) {
        addCommands(
            new ShooterCommand(systems, velocity),
            new SequentialCommandGroup(
                new WaitCommand(SHOOTER_WAIT_TILL_SPEED), 
                new ParallelCommandGroup(
                    new FeederBottomCommand(systems, false),
                    new FeederTopCommand(systems, false)
                )
            )
        );
    }

    public ShootCommand(Systems systems, boolean b) {
    }
}
