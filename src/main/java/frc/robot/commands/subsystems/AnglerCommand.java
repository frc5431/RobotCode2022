package frc.robot.commands.subsystems;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Systems;
import frc.robot.subsystems.Angler;
import frc.team5431.titan.core.misc.Logger;

public class AnglerCommand extends InstantCommand {
    private final Angler angler;
    private final boolean reverse;
    private final double deltaPos;

    public AnglerCommand(Systems systems, boolean reverse) {
        this(systems, Angler.DEFAULT_SPEED, reverse);
    }

    public AnglerCommand(Systems systems, double deltaPos) {
        this(systems, deltaPos, false);
    }

    public AnglerCommand(Systems systems, double deltaPos, boolean reverse) {
        this.angler = systems.getAngler();
        this.reverse = reverse;
        this.deltaPos = deltaPos;

        addRequirements(angler);
    }

    @Override
    public void initialize() {
        angler.change(reverse ? -deltaPos : deltaPos);
	}

    @Override
    public void end(boolean interrupted) {
        Logger.l("Intake Command Done");
    }
}
