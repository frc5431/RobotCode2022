package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Systems;
import frc.team5431.titan.core.leds.BlinkinPattern;

public class LEDCommand extends InstantCommand {
    public LEDCommand(Systems systems, double pwmValue) {
        super(() -> systems.getLed().set(pwmValue));
    }

    public LEDCommand(Systems systems, BlinkinPattern pattern) {
        super(() -> systems.getLed().set(pattern));
    }

    public static enum COMMAND {
        NEXT, PREV;
    }

    public LEDCommand(Systems systems, COMMAND cmd) {
        super(() -> {
            if (cmd == COMMAND.NEXT) systems.getLed().nextPattern();
            else systems.getLed().prevPattern();
        });
    }
}
