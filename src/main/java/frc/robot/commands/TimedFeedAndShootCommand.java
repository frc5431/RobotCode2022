package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Systems;
import frc.robot.commands.subsystems.ShooterCommand;
import frc.robot.subsystems.Shooter;
import frc.team5431.titan.core.misc.Calc;

public class TimedFeedAndShootCommand extends ParallelCommandGroup {
    public static final double FEEDER_PUSH_DOWN_DELAY = 0.4;
    public static final double MIN_SHOOTER_WAIT_TILL_SPEED = 0.9; // 0.7 // with 2 wheel 1.0 // before flywheel change: 0.25
    public static final double MAX_SHOOTER_WAIT_TILL_SPEED = 1.3; // 1.0 // with 2 wheel 1.8 // before flywheel change: 0.5
    public static final double FEEDER_BOTTOM_DELAY = 0.75; // 0.175

    private final Systems systems;

    public TimedFeedAndShootCommand(Systems systems, Shooter.Velocity velocity) {
        this(systems, velocity, true);
    }

    public TimedFeedAndShootCommand(Systems systems, Shooter.Velocity velocity, boolean waitForFlywheel) {
        this(systems, velocity.getVelocity(), waitForFlywheel);
    }

    public TimedFeedAndShootCommand(Systems systems, double velocity) {
        this(systems, velocity, true);
    }

    public TimedFeedAndShootCommand(Systems systems, double velocity, boolean waitForFlywheel) {
        this(systems, () -> velocity, waitForFlywheel);
    }

    public TimedFeedAndShootCommand(Systems systems, DoubleSupplier supplier) {
        this(systems, supplier, true);
    }

    public TimedFeedAndShootCommand(Systems systems, DoubleSupplier supplier, boolean waitForFlywheel) {
        this.systems = systems;
        addCommands(
            new LEDCommand(systems, Constants.LEDPATTERN_SHOOT),
            new SequentialCommandGroup(
                // new WaitUntilCommand(() -> !systems.getUpperFeederSensor().get()),
                new WaitCommand(FEEDER_PUSH_DOWN_DELAY),
                new ShooterCommand(systems, supplier)
            ),
            new SequentialCommandGroup(
                // new WaitUntilCommand(() -> !systems.getUpperFeederSensor().get())
                systems.getFeeder().getBottom().runFeederCommand(true).withTimeout(FEEDER_PUSH_DOWN_DELAY/2).andThen(
                    systems.getFeeder().getTop().runFeederCommand(true).withTimeout(FEEDER_PUSH_DOWN_DELAY/2)
                ).withTimeout(FEEDER_PUSH_DOWN_DELAY),
                waitForFlywheel
                    ? new WaitCommand(0.5).andThen( new WaitUntilCommand(() -> systems.getShooter().atVelocity()) )
                    : new WaitCommand(() -> Calc.map(supplier.getAsDouble(), 0, Shooter.MAX_VELOCITY, MIN_SHOOTER_WAIT_TILL_SPEED, MAX_SHOOTER_WAIT_TILL_SPEED)), 
                // new ParallelCommandGroup(
                //     new SequentialCommandGroup(
                //         // new WaitCommand(FEEDER_BOTTOM_DELAY),
                //         new WaitUntilCommand(() -> systems.getShooter().atVelocity()),
                //         new FeederBottomCommand(systems, false)
                //     ),
                //     new FeederTopWaitShooterCommand(systems, false)
                // )
                new ParallelCommandGroup(
                    // LEDs
                    new SequentialCommandGroup(
                        new LEDCommand(systems, Constants.LEDPATTERN_SHOOT_BB),
                        new WaitCommand(0.1),
                        new LEDCommand(systems, Constants.LEDPATTERN_SHOOT),
                        new WaitCommand(0.15),
                        new LEDCommand(systems, Constants.LEDPATTERN_SHOOT_BB),
                        new WaitCommand(0.1),
                        new LEDCommand(systems, Constants.LEDPATTERN_SHOOT)
                    ),
                    systems.getFeeder().getTop().runFeederCommand(false),
                    new WaitCommand(0.2).andThen(systems.getFeeder().getBottom().runFeederCommand(false))
                )
                // new FeedEverything(systems, false)
            )
        );
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        systems.getLed().set(Constants.LEDPATTERN_DEFAULT);
    }
}
