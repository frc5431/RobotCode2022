package frc.robot.commands.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Systems;
import frc.robot.subsystems.Climber;
import frc.team5431.titan.core.misc.Logger;

/**
 * @author Colin Wong
 */
public class ClimberExtendCommand extends CommandBase {
    private final Climber climber;
    private final boolean direction;
	private final double speed;

    public ClimberExtendCommand(Systems systems, boolean reverse) {
        this(systems, Climber.DEFAULT_SPEED_EXTEND, reverse);
    }

    public ClimberExtendCommand(Systems systems, double speed) {
        this(systems, speed, false);
    }

    public ClimberExtendCommand(Systems systems, double speed, boolean reverse) {
        this.climber = systems.getClimber();
        this.direction = reverse;
		this.speed = speed;

        addRequirements(climber.getExtend());
    }

    @Override
    public void initialize() {

	}
	
	@Override
	public void execute() {
        climber.setExtend(direction ? speed : -speed);
	}

    @Override
    public void end(boolean interrupted) {
        Logger.l("Climber Extend Command Done");
        climber.setExtend(0);
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}