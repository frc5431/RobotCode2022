package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Systems;
import frc.robot.commands.subsystems.FeederBottomCommand;
import frc.robot.commands.subsystems.FeederTopCommand;
import frc.robot.commands.subsystems.ShooterCommand;
import frc.robot.subsystems.Shooter;
import frc.team5431.titan.core.misc.Calc;

public class ShootCommand extends ParallelCommandGroup {
    public static final double FEEDER_PUSH_DOWN_DELAY = 0.4;
    public static final double MIN_SHOOTER_WAIT_TILL_SPEED = 0.9; // 0.7 // with 2 wheel 1.0 // before flywheel change: 0.25
    public static final double MAX_SHOOTER_WAIT_TILL_SPEED = 1.3; // 1.0 // with 2 wheel 1.8 // before flywheel change: 0.5
    public static final double FEEDER_BOTTOM_DELAY = 0.75; // 0.175

    public ShootCommand(Systems systems, Shooter.Velocity velocity) {
        this(systems, velocity, true);
    }

    public ShootCommand(Systems systems, Shooter.Velocity velocity, boolean waitForFlywheel) {
        this(systems, velocity.getVelocity(), waitForFlywheel);
    }

    public ShootCommand(Systems systems, double velocity) {
        this(systems, velocity, true);
    }

    public ShootCommand(Systems systems, double velocity, boolean waitForFlywheel) {
        this(systems, () -> velocity, waitForFlywheel);
    }

    public ShootCommand(Systems systems, DoubleSupplier supplier) {
        this(systems, supplier, true);
    }

    public ShootCommand(Systems systems, DoubleSupplier supplier, boolean waitForFlywheel) {
        addCommands(
            new SequentialCommandGroup(
                // new WaitUntilCommand(() -> !systems.getUpperFeederSensor().get()),
                new WaitCommand(FEEDER_PUSH_DOWN_DELAY),
                new ShooterCommand(systems, supplier)
            ),
            new SequentialCommandGroup(
                // new WaitUntilCommand(() -> !systems.getUpperFeederSensor().get())
                new WaitCommand(FEEDER_PUSH_DOWN_DELAY)
                    .deadlineWith(new SequentialCommandGroup(
                        new WaitCommand(FEEDER_PUSH_DOWN_DELAY/2)
                            .deadlineWith(new FeederBottomCommand(systems, true)),
                        new WaitCommand(FEEDER_PUSH_DOWN_DELAY/2)
                            .deadlineWith(new FeederTopCommand(systems, true))
                    )),
                new WaitCommand(() -> Calc.map(supplier.getAsDouble(), 0, Shooter.MAX_VELOCITY, MIN_SHOOTER_WAIT_TILL_SPEED, MAX_SHOOTER_WAIT_TILL_SPEED)), 
                new WaitUntilCommand(() -> (systems.getShooter().atVelocity() || !waitForFlywheel)),
                // new ParallelCommandGroup(
                //     new SequentialCommandGroup(
                //         // new WaitCommand(FEEDER_BOTTOM_DELAY),
                //         new WaitUntilCommand(() -> systems.getShooter().atVelocity()),
                //         new FeederBottomCommand(systems, false)
                //     ),
                //     new FeederTopWaitShooterCommand(systems, false)
                // )
                new FeedEverything(systems, false)
            )
        );
    }
}
