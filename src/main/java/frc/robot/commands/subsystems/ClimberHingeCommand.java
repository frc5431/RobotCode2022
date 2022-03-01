package frc.robot.commands.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Systems;
import frc.robot.subsystems.Climber;
import frc.team5431.titan.core.misc.Logger;

/**
 * @author Colin Wong
 */
public class ClimberHingeCommand extends CommandBase {
    private final Climber climber;
    private final boolean direction;
	private final double speed;

    public ClimberHingeCommand(Systems systems, boolean reverse) {
        this(systems, Climber.DEFAULT_SPEED_HINGE, reverse);
    }

    public ClimberHingeCommand(Systems systems, double speed) {
        this(systems, speed, false);
    }

    public ClimberHingeCommand(Systems systems, double speed, boolean reverse) {
        this.climber = systems.getClimber();
        this.direction = reverse;
		this.speed = speed;

        addRequirements(climber.getHinge());
    }

    @Override
    public void initialize() {

	}
	
	@Override
	public void execute() {
        climber.setHinge(direction ? speed : -speed);
	}

    @Override
    public void end(boolean interrupted) {
        Logger.l("Climber Hinge Command Done");
        climber.setHinge(0);
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}