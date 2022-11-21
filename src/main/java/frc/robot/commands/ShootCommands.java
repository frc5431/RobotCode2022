package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Systems;
import frc.robot.commands.subsystems.AnglerCommand;
import frc.robot.util.CameraCalc;

import static edu.wpi.first.wpilibj2.command.Commands.*;

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
            new TimedFeedAndShootCommand(systems, preset.getRpm(), waitForFlywheel)
        );
    }

    public static Command shootCalcRPMCommand(Systems systems) {
        return shootCalcRPMCommand(systems, true);
    }

    public static Command shootCalcRPMCommand(Systems systems, boolean waitForFlywheel) {
        return new TimedFeedAndShootCommand(systems, () -> CameraCalc.calculateRPM(systems.getCamera()), waitForFlywheel);
    }
}
