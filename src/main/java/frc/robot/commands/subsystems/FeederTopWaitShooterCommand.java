package frc.robot.commands.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Systems;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;
import frc.team5431.titan.core.misc.Logger;

/**
 * @author Colin Wong
 */
public class FeederTopWaitShooterCommand extends CommandBase {
    private final Feeder feeder;
    private final Shooter shooter;
    private final DigitalInput dio;
    private final boolean reverse;
	private final double speed;

    public FeederTopWaitShooterCommand(Systems systems, boolean reverse) {
        this(systems, Feeder.DEFAULT_SPEED_TOP, reverse);
    }

    public FeederTopWaitShooterCommand(Systems systems, double speed) {
        this(systems, speed, false);
    }

    public FeederTopWaitShooterCommand(Systems systems, double speed, boolean reverse) {
        this.feeder = systems.getFeeder();
        this.shooter = systems.getShooter();
        this.dio = systems.getUpperFeederSensor();
        this.reverse = reverse;
		this.speed = speed;

        addRequirements(feeder.getTop());
    }

    @Override
    public void initialize() {

	}
	
	@Override
	public void execute() {
        if (dio.get() || shooter.atVelocity()) {
            Logger.l("DIO: %s  Shooter: %s", dio.get(), shooter.atVelocity());
            feeder.setTop(reverse ? -speed : speed);
        } else
            feeder.setTop(0);
	}

    @Override
    public void end(boolean interrupted) {
        Logger.l("Feeder Top Command Done");
        feeder.setTop(0);
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}