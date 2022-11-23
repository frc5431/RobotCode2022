package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Systems;
import frc.robot.commands.subsystems.AnglerCommand;
import frc.robot.commands.subsystems.ShooterCommand;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Shooter;
import frc.robot.util.CameraCalc;
import frc.team5431.titan.core.misc.Calc;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import java.util.function.DoubleSupplier;

public class ShootCommands {

    public static enum ShootPresets {
        HUB(0.25, 12750), SAFEZONE(0.675, 12626), TERMINAL(0.8, 15000);

        private double anglerPos;
        private double rpm;

        ShootPresets(double anglerPos, double rpm) {
            this.anglerPos = anglerPos;
            this.rpm = rpm;
        }

        public double getAnglerPos() {
            return anglerPos;
        }

        public double getRpm() {
            return rpm;
        }
    }

    public static Command aimAndShootCommand(Systems systems) {
        return aimAndShootCommand(systems, true);
    }

    public static Command aimAndShootCommand(Systems systems, boolean waitForFlywheel) {
        final double AIM_LENGTH = 0.25;
    
        return parallel(
            new AimCommand(systems, true).withTimeout(AIM_LENGTH).andThen(shootCalcRPMCommand(systems, waitForFlywheel)),
            systems.getIntake().runIntakeCommand(false)
        );
    }

    public static Command angleAndShootCommand(Systems systems, ShootPresets preset) {
        return angleAndShootCommand(systems, preset, true);
    }

    public static Command angleAndShootCommand(Systems systems, ShootPresets preset, boolean waitForFlywheel) {
        return parallel(
            new AnglerCommand(systems, AnglerCommand.COMMAND.SET, preset.getAnglerPos()),
            timedFeedShootWithAimCommand(systems, preset.getRpm(), false, waitForFlywheel)
        );
    }

    public static Command shootCalcRPMCommand(Systems systems) {
        return shootCalcRPMCommand(systems, true);
    }

    public static Command shootCalcRPMCommand(Systems systems, boolean waitForFlywheel) {
        return timedFeedShootWithAimCommand(systems, () -> CameraCalc.calculateRPM(systems.getCamera()), false, waitForFlywheel);
    }

    private static final double FEEDER_PUSH_DOWN_DELAY = 0.4;
    private static final double MIN_SHOOTER_WAIT_TILL_SPEED = 0.9; // 0.7 // with 2 wheel 1.0 // before flywheel change: 0.25
    private static final double MAX_SHOOTER_WAIT_TILL_SPEED = 1.3; // 1.0 // with 2 wheel 1.8 // before flywheel change: 0.5

    public static Command timedFeedShootWithAimCommand(Systems systems, double velocity, boolean shouldAim, boolean waitForFlywheel) {
        return timedFeedShootWithAimCommand(systems, () -> velocity, shouldAim, waitForFlywheel);
    }

    public static Command timedFeedShootWithAimCommand(Systems systems, DoubleSupplier supplier, boolean shouldAim, boolean waitForFlywheel) {
        return parallel(
            new LEDCommand(systems, Constants.LEDPATTERN_SHOOT),
            sequence(
                new WaitCommand(FEEDER_PUSH_DOWN_DELAY),
                new ShooterCommand(systems, supplier)
            ),
            sequence(
                parallel(
                    systems.getFeeder().getBottom().runFeederCommand(true).withTimeout(FEEDER_PUSH_DOWN_DELAY/2).andThen(
                        systems.getFeeder().getTop().runFeederCommand(true).withTimeout(FEEDER_PUSH_DOWN_DELAY/2)
                    ).withTimeout(FEEDER_PUSH_DOWN_DELAY),
                    either(new AimCommand(systems, true).asProxy(), none(), () -> (shouldAim && !Drivebase.lockedToHub))
                ),
                waitForFlywheel
                    ? new WaitCommand(0.85).andThen( waitUntil(() -> systems.getShooter().atVelocity()) )
                    : new WaitCommand(() -> Calc.map(supplier.getAsDouble(), 0, Shooter.MAX_VELOCITY, MIN_SHOOTER_WAIT_TILL_SPEED, MAX_SHOOTER_WAIT_TILL_SPEED)), 
                parallel(
                    // LEDs
                    sequence(
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
            )
        ).finallyDo((interrupted) -> systems.getLed().set(Constants.LEDPATTERN_DEFAULT));
    }
}
