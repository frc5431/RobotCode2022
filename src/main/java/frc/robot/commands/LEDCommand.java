package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Systems;

public class LEDCommand extends InstantCommand {
    public LEDCommand(Systems systems, double pwmValue) {
        super(() -> systems.getLed().set(pwmValue));
    }
}
