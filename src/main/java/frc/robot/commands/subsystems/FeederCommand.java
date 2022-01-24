package frc.robot.commands.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Systems;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;
import frc.team5431.titan.core.misc.Logger;

/**
 * @author Ryan Hirasaki
 */
public class FeederCommand extends CommandBase {
    private final Feeder feeder;
    private final boolean direction;
	private final double speed;
	private final Shooter shooter;
	private boolean rpmWait;  


    public FeederCommand(Systems systems, double speed,  boolean rpmWait) {
        this(systems, speed, false, rpmWait);
    }

    public FeederCommand(Systems systems, double speed, boolean reverse, boolean rpmWait) {
        this.feeder = systems.getFeeder();
        this.direction = reverse;
		this.speed = speed;
		this.shooter = systems.getShooter();
		this.rpmWait = rpmWait; 
		 

        addRequirements(feeder);
    }

    @Override
    public void initialize() {

	}
	
	@Override
	public void execute() {

		if(direction) {

			if (shooter.atVelocity() || !rpmWait) {
				feeder.set(speed);
			} else {
				Logger.l("Shooter not at speed, not pushing up!");
				feeder.set(0);
			}
		}
		else {
			feeder.set(-speed);
		}
	}

    @Override
    public void end(boolean interrupted) {
        Logger.l("Feeder Command Done");
        feeder.set(0);
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}