package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Systems;
import frc.robot.commands.subsystems.ClimberExtendCommand;
import frc.robot.commands.subsystems.ClimberHingeCommand;

// TODO: add timings
// visual cues with LEDs to indicate stage of auto climb
// pause? after each rung waiting for operator permission to continue to next rung
// ^ maybe press climb button again?
// maybe button with lid
// use match time to stop auto climb if in unstable position
/**
 * Time-based autonomous climb
 * @author Colin Wong
 */
public class AutoClimbCommand extends SequentialCommandGroup {
    public AutoClimbCommand(Systems systems) {
        addCommands(
            new WaitCommand(0) // Extend climber 
                .deadlineWith(new ClimberExtendCommand(systems, false)),
            // Drive here, maybe make above part manual or have "continue" feature?
            new WaitCommand(0) // Retract climber, pulls robot up past stationary hooks
                .deadlineWith(new ClimberExtendCommand(systems, true)), // mid rung latch
            new WaitCommand(0) // Sets stationary hooks on rungs, extends past stationary hooks
                .deadlineWith(new ClimberExtendCommand(systems, false)), // mid rung stationary
            new WaitCommand(0) // Positions hooks near high rung
                .deadlineWith(new ParallelCommandGroup(
                    new ClimberHingeCommand(systems, true),
                    new ClimberExtendCommand(systems, false)
                )),
            new WaitCommand(0) // Pivot in hooks to contact high rung WITHOUT PUSHING ROBOT OFF
                .deadlineWith(new ClimberHingeCommand(systems, false)),
            new WaitCommand(0) // Retract climber, latch onto high rung, stationary hooks off
                .deadlineWith(new ClimberExtendCommand(systems, true)), // high rung latch
            new WaitCommand(0) // Return rung to go under stationary hooks
                .deadlineWith(new ParallelCommandGroup(
                    new ClimberHingeCommand(systems, false),
                    new ClimberExtendCommand(systems, true)
                )),
            new WaitCommand(0) // Extend hooks all the way, latch stationary onto high rung
                .deadlineWith(new ClimberExtendCommand(systems, false)), // high rung stationary !SWINGING!
            new WaitUntilCommand(() -> true), // systems.getDrivebase().getGyro()), // Wait until robot is swinging towards driver station
            new WaitCommand(0) // Hooks contact traversal rung
                .deadlineWith(new ClimberHingeCommand(systems, true)),
            new WaitCommand(0)
                .deadlineWith(new ClimberExtendCommand(systems, true)), // traversal rung lock
            new WaitCommand(0)
                .deadlineWith(new ClimberHingeCommand(systems, false))
        );
    }
}
