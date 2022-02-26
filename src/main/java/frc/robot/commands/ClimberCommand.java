package frc.robot.commands;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Systems;
import frc.robot.subsystems.Climber;
//import frc.robot.subsystems.Elevator;

public class ClimberCommand {


/**
 * @author Aahana Shrivastava
 */
public class DefaultElevator extends CommandBase {

    private final Climber elevator;
    private final DoubleSupplier pow;

    public DefaultElevator(Systems systems, DoubleSupplier power) {
        this.elevator = systems.getClimber(); 
        this.pow = power;

        addRequirements(elevator);
    }

    @Override
    public void execute() {
        elevator.setSpeed(pow.getAsDouble());
    }
}
}
