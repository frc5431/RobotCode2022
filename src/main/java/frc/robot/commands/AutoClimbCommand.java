package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Systems;

// TODO: visual cues with LEDs to indicate stage of auto climb
// maybe button with lid
// use match time to stop auto climb if in unstable position
/**
 * Time-based autonomous climb
 * @author Colin Wong
 */
public class AutoClimbCommand extends SequentialCommandGroup {
    public AutoClimbCommand(Systems systems, BooleanSupplier continueSupplier) {
        addCommands(
            new WaitCommand(1.5) // Extend climber 
                .deadlineWith(systems.getClimber().getExtend().runClimberCommand(false)),
            new WaitUntilCommand(continueSupplier),
            // Drive here, maybe make above part manual or have "continue" feature?
            new WaitCommand(2.5) // Retract climber, pulls robot up past stationary hooks
                .deadlineWith(new ParallelCommandGroup(
                    systems.getClimber().getExtend().runClimberCommand(true),
                    systems.getClimber().getHinge().runClimberCommand(-0.4)
                )), // mid rung latch
            new WaitCommand(0.5) // Sets stationary hooks on rungs, extends past stationary hooks
                .deadlineWith(systems.getClimber().getExtend().runClimberCommand(false)), // mid rung stationary
            new WaitCommand(1.1) // Positions hooks near high rung
                .deadlineWith(new ParallelCommandGroup(
                    systems.getClimber().getHinge().runClimberCommand(false),
                    systems.getClimber().getExtend().runClimberCommand(0.8)
                )),
            new WaitCommand(0.3) // Fully extends climber
                .deadlineWith(systems.getClimber().getExtend().runClimberCommand(false)),
            new WaitUntilCommand(continueSupplier),
            new WaitCommand(0.5) // Pivot in hooks to contact high rung WITHOUT PUSHING ROBOT OFF
                .deadlineWith(systems.getClimber().getHinge().runClimberCommand(true)),
            new WaitUntilCommand(continueSupplier),
            new WaitCommand(0.65) // Retract climber, latch onto high rung, stationary hooks off
                .deadlineWith(systems.getClimber().getExtend().runClimberCommand(true)), // high rung latch
            new WaitCommand(2.7) // Return rung to go under stationary hooks
                .deadlineWith(new ParallelCommandGroup(
                    systems.getClimber().getExtend().runClimberCommand(-0.9),
                    systems.getClimber().getHinge().runClimberCommand(true)
                )),
            new WaitCommand(2.8) // Extend hooks all the way, latch stationary onto high rung
                .deadlineWith(systems.getClimber().getExtend().runClimberCommand(false)), // high rung stationary !SWINGING!
            new WaitUntilCommand(continueSupplier),
            new WaitUntilCommand(() -> systems.getDrivebase().getGyro().getRoll() > 0), // Wait until robot is swinging towards driver station
            new WaitUntilCommand(() -> systems.getDrivebase().getGyro().getRoll() < -15),
            new WaitCommand(1.0) // Hooks contact traversal rung
                .deadlineWith(systems.getClimber().getHinge().runClimberCommand(false)),
            new WaitCommand(1.5)
                .deadlineWith(systems.getClimber().getExtend().runClimberCommand(true)), // traversal rung lock
            new WaitCommand(2.0)
                .deadlineWith(systems.getClimber().getHinge().runClimberCommand(true))
        );
    }
}
