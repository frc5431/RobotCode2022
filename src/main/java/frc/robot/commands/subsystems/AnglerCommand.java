package frc.robot.commands.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Systems;
import frc.robot.subsystems.Angler;
import frc.team5431.titan.core.misc.Logger;

public class AnglerCommand extends CommandBase {
    private final Angler angler;
    private final COMMAND command;
    private final boolean reverse;
    private final DoubleSupplier supplier;

    public static enum COMMAND {
        CHANGE, SET
    }

    public AnglerCommand(Systems systems, boolean reverse) {
        this(systems, Angler.DEFAULT_SPEED, reverse);
    }

    public AnglerCommand(Systems systems, double deltaPos) {
        this(systems, deltaPos, false);
    }

    public AnglerCommand(Systems systems, double deltaPos, boolean reverse) {
        this(systems, COMMAND.CHANGE, () -> deltaPos, reverse);
    }

    public AnglerCommand(Systems systems, COMMAND command, double value) {
        this(systems, command, () -> value);
    }

    public AnglerCommand(Systems systems, COMMAND command, DoubleSupplier supplier) {
        this(systems, command, supplier, false);
    }

    public AnglerCommand(Systems systems, COMMAND command, DoubleSupplier supplier, boolean reverse) {
        this.angler = systems.getAngler();
        this.command = command;
        this.reverse = reverse;
        this.supplier = supplier;

        addRequirements(angler);
    }

    @Override
    public void initialize() {
        if (command == COMMAND.CHANGE)
            angler.change(reverse ? -supplier.getAsDouble() : supplier.getAsDouble());
        else if (command == COMMAND.SET) {
            if (supplier.getAsDouble() >= 0)
                angler.set(supplier.getAsDouble());
        }
	}

    @Override
    public void execute() {
        if (command == COMMAND.SET) {
            if (supplier.getAsDouble() >= 0)
                angler.set(supplier.getAsDouble());
        }
    }

    @Override
    public void end(boolean interrupted) {
        Logger.l("Intake Command Done");
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
