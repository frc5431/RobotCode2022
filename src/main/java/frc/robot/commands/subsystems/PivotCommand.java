package frc.robot.commands.subsystems;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Systems;
import frc.robot.subsystems.Pivot;
import frc.team5431.titan.core.misc.Logger;

/**
 * @author Ryan Hirasaki
 */
public class PivotCommand extends CommandBase {
    private final Pivot pivot;
   // private final boolean interruptible;

    public PivotCommand(Systems systems) {
        this(systems, true);
    }

    public PivotCommand(Systems systems, boolean interruptible) {
        this.pivot = systems.getPivot();
        //this.position = pos;
       // this.interruptible = interruptible;


        addRequirements(pivot);
    }

    @Override
    public void initialize() {
        Logger.l("Running Pivot Command!");
        //pivot.setPivotLocation(position);
    }

    @Override
    public void end(boolean interrupted) {
        Logger.l("Finished Pivot Command!");
    }

    //@Override
    //public boolean isFinished() {
        //return Calc.approxEquals(pivot.error(), position.getValue(), 500);
    //}

    @Override
    public void schedule(boolean interruptible) {
        //CommandScheduler.getInstance().schedule(interruptible && this.interruptible, this); // Ensures that the command will not be interruptble if either we or the bult-in system declare it
    }
}