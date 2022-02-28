package frc.robot.commands.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Systems;
import frc.robot.subsystems.Feeder;
import frc.team5431.titan.core.misc.Logger;

/**
 * @author Colin Wong
 */
public class FeederTopCommand extends CommandBase {
    private final Feeder feeder;
    private final boolean direction;
	private final double speed;

    public FeederTopCommand(Systems systems, boolean reverse) {
        this(systems, Feeder.DEFAULT_SPEED, reverse);
    }

    public FeederTopCommand(Systems systems, double speed) {
        this(systems, speed, false);
    }

    public FeederTopCommand(Systems systems, double speed, boolean reverse) {
        this.feeder = systems.getFeeder();
        this.direction = reverse;
		this.speed = speed;

        addRequirements(feeder.getTop());
    }

    @Override
    public void initialize() {

	}
	
	@Override
	public void execute() {
        feeder.setTop(direction ? speed : -speed);
	}

    @Override
    public void end(boolean interrupted) {
        Logger.l("Feeder Command Done");
        feeder.setTop(0);
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}