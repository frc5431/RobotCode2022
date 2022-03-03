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
    private final boolean direction;
    private final double speed;
   // private final boolean interruptible;

    public PivotCommand(Systems systems, double speed) {
        this(systems, speed, false);
    }

    public PivotCommand(Systems systems, boolean reverse) {
        this(systems, Pivot.DEFAULT_SPEED, reverse);
    }

    public PivotCommand(Systems systems, double speed, boolean reverse) {
        this.pivot = systems.getPivot();
        this.direction = reverse;
        this.speed = speed;
        
        addRequirements(pivot);
    }

    @Override
    public void initialize() {
        Logger.l("Running Pivot Command!");
        //pivot.setPivotLocation(position);
        pivot.set(direction ? speed : -speed);
    }

    @Override
    public void end(boolean interrupted) {
        Logger.l("Finished Pivot Command!");
        pivot.set(0);
    }

    //@Override
    //public boolean isFinished() {
        //return Calc.approxEquals(pivot.error(), position.getValue(), 500);
    //}
}