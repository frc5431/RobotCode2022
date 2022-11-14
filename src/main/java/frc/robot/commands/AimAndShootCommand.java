package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Systems;

public class AimAndShootCommand extends ParallelCommandGroup {
    public static final double AIM_LENGTH = 0.25;

    public AimAndShootCommand(Systems systems) {
        this(systems, true);
    }

    public AimAndShootCommand(Systems systems, boolean waitForFlywheel) {
        addCommands(
            new WaitCommand(AIM_LENGTH)
                .deadlineWith(new AimCommand(systems, true)),
            new WaitCommand(AIM_LENGTH)
                .andThen(new ShootWithCalcRPMCommand(systems, waitForFlywheel)),
            systems.getIntake().runIntakeCommand(false)
        );
    }
}
