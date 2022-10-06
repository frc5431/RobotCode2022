package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Systems;
import frc.robot.commands.subsystems.ClimberExtendCommand;
import frc.robot.commands.subsystems.ClimberHingeCommand;

// TODO: add timings
// gyro check
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
    public AutoClimbCommand(Systems systems, BooleanSupplier continueSupplier) {
        addCommands(
            new WaitCommand(1.5) // Extend climber 
                .deadlineWith(new ClimberExtendCommand(systems, false)),
            new WaitUntilCommand(continueSupplier),
            // Drive here, maybe make above part manual or have "continue" feature?
            new WaitCommand(2.5) // Retract climber, pulls robot up past stationary hooks
                .deadlineWith(new ClimberExtendCommand(systems, true)), // mid rung latch
            new WaitCommand(0.5) // Sets stationary hooks on rungs, extends past stationary hooks
                .deadlineWith(new ClimberExtendCommand(systems, false)), // mid rung stationary
            new WaitCommand(1.1) // Positions hooks near high rung
                .deadlineWith(new ParallelCommandGroup(
                    new ClimberHingeCommand(systems, true),
                    new ClimberExtendCommand(systems, 0.8)
                )),
            new WaitCommand(0.8) // Fully extends climber
                .deadlineWith(new ClimberExtendCommand(systems, false)),
            new WaitUntilCommand(continueSupplier),
            new WaitCommand(0.38) // Pivot in hooks to contact high rung WITHOUT PUSHING ROBOT OFF
                .deadlineWith(new ClimberHingeCommand(systems, false)),
            new WaitUntilCommand(continueSupplier),
            new WaitCommand(0.65) // Retract climber, latch onto high rung, stationary hooks off
                .deadlineWith(new ClimberExtendCommand(systems, true)), // high rung latch
            new WaitCommand(2.7) // Return rung to go under stationary hooks
                .deadlineWith(new ParallelCommandGroup(
                    new ClimberExtendCommand(systems, -0.4),
                    new ClimberHingeCommand(systems, false)
                )),
            new WaitCommand(2.8) // Extend hooks all the way, latch stationary onto high rung
                .deadlineWith(new ClimberExtendCommand(systems, false)), // high rung stationary !SWINGING!
            new WaitUntilCommand(continueSupplier),
            new WaitUntilCommand(() -> true), // systems.getDrivebase().getGyro()), // Wait until robot is swinging towards driver station
            new WaitUntilCommand(() -> true), // systems.getDrivebase().getGyro()),
            new WaitCommand(1.0) // Hooks contact traversal rung
                .deadlineWith(new ClimberHingeCommand(systems, true)),
            new WaitCommand(1.5)
                .deadlineWith(new ClimberExtendCommand(systems, true)), // traversal rung lock
            new WaitCommand(2.0)
                .deadlineWith(new ClimberHingeCommand(systems, false))
        );
    }
}
