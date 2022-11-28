package frc.robot.subsystems.base;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

public interface Calibratable {
    
    void setCalibrateMode(boolean value);
    void resetEncoder();

    default Command calibrateCommand() {
        return new StartEndCommand(
            () -> this.setCalibrateMode(true),
            () -> {
                this.setCalibrateMode(false);
                this.resetEncoder();
            }
        );
    }
}
