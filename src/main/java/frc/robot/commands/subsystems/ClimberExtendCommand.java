package frc.robot.commands.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Systems;
import frc.robot.subsystems.Climber;
import frc.team5431.titan.core.leds.Blinkin;
import frc.team5431.titan.core.misc.Logger;

/**
 * @author Colin Wong
 */
public class ClimberExtendCommand extends CommandBase {
    private final Climber climber;
    private final boolean reverse;
	private final DoubleSupplier supplier;

    private final Blinkin leds;

    /**
     * @param systems systems of robot
     * @param reverse false: extend mechanism; true: retract mechanism
     */
    public ClimberExtendCommand(Systems systems, boolean reverse) {
        this(systems, () -> Climber.DEFAULT_SPEED_EXTEND, reverse);
    }

    /**
     * @param systems systems of robot
     * @param speed speed of climber extension mechanism
     */
    public ClimberExtendCommand(Systems systems, double speed) {
        this(systems, () -> speed, false);
    }

    /**
     * @param systems systems of robot
     * @param supplier speed of climber extension mechanism
     */
    public ClimberExtendCommand(Systems systems, DoubleSupplier supplier) {
        this(systems, supplier, false);
    }

    /**
     * @param systems systems of robot
     * @param supplier speed of climber extension mechanism
     * @param reverse false: extend mechanism; true: retract mechanism
     */
    public ClimberExtendCommand(Systems systems, DoubleSupplier supplier, boolean reverse) {
        this.climber = systems.getClimber();
        this.reverse = reverse;
		this.supplier = supplier;
        this.leds = systems.getLed();

        addRequirements(climber.getExtend());
    }

    @Override
    public void initialize() {

	}
	
	@Override
	public void execute() {
        double value = supplier.getAsDouble();
        climber.setExtend(reverse ? -value : value);
        if (Math.abs(value) > 0.01) {
            leds.set(Constants.LEDPATTERN_CLIMB);
        }
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