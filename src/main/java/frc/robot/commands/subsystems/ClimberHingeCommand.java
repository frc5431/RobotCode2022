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
    private final boolean reverse;
	private final double speed;

    /**
     * @param systems systems of robot
     * @param reverse false: pivot in mechanism; true: pivot out mechanism
     */
    public ClimberHingeCommand(Systems systems, boolean reverse) {
        this(systems, Climber.DEFAULT_SPEED_HINGE, reverse);
    }

    /**
     * @param systems systems of robot
     * @param speed speed of climber pivot/hinge mechanism
     */
    public ClimberHingeCommand(Systems systems, double speed) {
        this(systems, speed, false);
    }

    /**
     * @param systems systems of robot
     * @param speed speed of climber pivot/hinge mechanism
     * @param reverse false: pivot in mechanism; true: pivot out mechanism
     */
    public ClimberHingeCommand(Systems systems, double speed, boolean reverse) {
        this.climber = systems.getClimber();
        this.reverse = reverse;
		this.speed = speed;

        addRequirements(climber.getHinge());
    }

    @Override
    public void initialize() {

	}
	
	@Override
	public void execute() {
        climber.setHinge(reverse ? -speed : speed);
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