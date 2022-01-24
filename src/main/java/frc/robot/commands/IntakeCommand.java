package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Systems;
import frc.robot.subsystems.Intake;
import frc.team5431.titan.core.misc.Logger;

public class IntakeCommand extends CommandBase {
    private final Intake intake;
    private final boolean direction;
    private final double speed;

    public IntakeCommand(Systems systems, boolean reverse) {
        this(systems, 1.0, reverse);
    }

    public IntakeCommand(Systems systems, double speed) {
        this(systems, speed, false);
    }

    public IntakeCommand(Systems systems, double speed, boolean reverse) {
        this.intake = systems.getIntake();
        this.direction = reverse;
        this.speed = speed;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setSpeed(direction ? speed : -speed);
	}

    @Override
    public void end(boolean interrupted) {
        Logger.l("Intake Command Done");
        intake.setSpeed(0);
    }
}
