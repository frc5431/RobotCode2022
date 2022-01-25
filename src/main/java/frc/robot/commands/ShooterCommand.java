package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Systems;
import frc.robot.subsystems.Shooter;
import frc.team5431.titan.core.misc.Logger;

/**
 * @author Ryan Hirasaki
 */
public class ShooterCommand extends CommandBase {

	public static boolean KILL = false;

    private final Shooter shooter;
    // private final Shooter.Speeds speed;    

    // public ShooterCommand(Systems systems, Shooter.Speeds speed) {
    //     this.shooter = systems.getShooter();
    //     this.speed = speed;
    //     this.velocity = null;

    //     addRequirements(shooter);
    // }

    public ShooterCommand(Systems systems) {
        this.shooter = systems.getShooter();
        // this.speed = null;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
		KILL = false;
        // if (speed == null)
            shooter.set(1);
        // if (velocity == null)
        //     shooter.set(speed);
    }

    @Override
    public void end(boolean interrupted) {
		Logger.l("Shooter Command Done");
		shooter.set(0);
	}
	
	@Override
	public boolean isFinished() {
		if(KILL) {
			KILL = false;
			return true;
		}
		else {
			return false;
		}
	}


}