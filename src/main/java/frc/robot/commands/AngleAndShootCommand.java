package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Systems;
import frc.robot.commands.subsystems.AnglerCommand;

/**
 * A manual version of AimAndShoot (i.e., no limelight)
 * @author Colin Wong
 */
public class AngleAndShootCommand extends ParallelCommandGroup {
    public static enum Position {
        HUB(0.25, 12750), SAFEZONE(0.675, 12626), TERMINAL(0.8, 15000);

        private double anglerPos;
        private double rpm;

        Position(double anglerPos, double rpm) {
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
    
    public AngleAndShootCommand(Systems systems, Position pos) {
        this(systems, pos, true);
    }

    public AngleAndShootCommand(Systems systems, Position pos, boolean waitForFlywheel) {
        addCommands(
            new AnglerCommand(systems, AnglerCommand.COMMAND.SET, pos.getAnglerPos()),
            new TimedFeedAndShootCommand(systems, pos.getRpm(), waitForFlywheel)
        );
    }
}
