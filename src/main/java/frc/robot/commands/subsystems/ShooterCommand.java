package frc.robot.commands.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Systems;
import frc.robot.subsystems.Shooter;
import frc.team5431.titan.core.misc.Logger;

/**
 * @author Colin Wong
 */
public class ShooterCommand extends CommandBase {

    private final Shooter shooter;
    private final DoubleSupplier velocitySupplier;

    public static DoubleSupplier supplierToTab = () -> 0;

    public ShooterCommand(Systems systems, Shooter.Velocity velocity) {
        this(systems, velocity.getVelocity());
    }

    public ShooterCommand(Systems systems, double velocity) {
        this(systems, () -> velocity);
    }

    public ShooterCommand(Systems systems, DoubleSupplier supplier) {
        this.shooter = systems.getShooter();
        this.velocitySupplier = supplier;
        System.out.println("setting to " + supplier.getAsDouble());

        supplierToTab = supplier;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        shooter.set(velocitySupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
		Logger.l("Shooter Command Done");
		shooter.set(Shooter.Velocity.OFF);
	}
	
	@Override
	public boolean isFinished() {
		return false;
	}

    public static DoubleSupplier getSupplierToTab() {
        return supplierToTab;
    }
}