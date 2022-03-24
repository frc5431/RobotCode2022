package frc.robot.commands;

import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Systems;
import frc.robot.subsystems.Drivebase;
import frc.robot.util.CameraCalc;
import frc.team5431.titan.core.misc.Logger;

public class AimCommand extends CommandBase {
    private final Drivebase drivebase;
    private final PhotonCamera camera;

    private final ProfiledPIDController turnPID;
    private final Timer timer;
    private static final double MIN_DURATION = 0.25;
    private static final double MIN_POWER = 0.1;

    private boolean lostTarget = false;

    public AimCommand(Systems systems) {
        this.drivebase = systems.getDrivebase();
        this.camera = systems.getCamera();
        this.turnPID = new ProfiledPIDController(
                1, 
                0, 
                0.01, 
                new Constraints(
                        Drivebase.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 
                        Drivebase.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
                )
        );
        this.turnPID.setGoal(0);
        this.turnPID.setTolerance(0.05, 0.2);

        this.timer = new Timer();

        try {
            Constants.tab_subsystems.addNumber("Turn PID Error", () -> {
                try {
                    return this.turnPID.getPositionError();
                } catch (Exception e) {
                    return 0;
                }
            });
            Constants.tab_subsystems.addBoolean("Is Vision Done?", this::isFinished);
        } catch (Exception e) {}

        addRequirements(drivebase);
    }

    @Override
    public void initialize() {
        camera.setDriverMode(false);
        camera.setLED(VisionLEDMode.kOn);
        timer.reset();
        timer.start();
        turnPID.reset(0);
        lostTarget = false;
    }

    @Override
    public void execute() {
        PhotonPipelineResult result = camera.getLatestResult();

        if (result.hasTargets()) {
            lostTarget = false;
            Logger.l("Meters to target: " + CameraCalc.getDistanceMeters(camera));

            double yawToTargetRadians = Units.degreesToRadians(result.getBestTarget().getYaw());

            double calculatedValue = 2*turnPID.calculate(yawToTargetRadians);

            Logger.l("Aim calc: %s -> %s", yawToTargetRadians, calculatedValue);
            Logger.l("Turn PID State: %s", turnPID.getPositionError());

            if (Math.abs(calculatedValue) <= MIN_POWER) {
                drivebase.stop();
            } else {
                drivebase.driveRaw(new ChassisSpeeds(0, 0, 
                        Math.copySign(
                            Math.max(
                                Math.abs(calculatedValue),
                                Drivebase.MIN_ANGULAR_VELOCITY
                            ), 
                            calculatedValue
                        )
                    )
                );
            }
        } else {
            // lostTarget = true;
            drivebase.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        camera.setDriverMode(Constants.DRIVER_MODE);
        camera.setLED(Constants.DEFAULT_LED_MODE);
        timer.stop();
        Logger.l("Aim Command ending - lost? %s", lostTarget);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(MIN_DURATION) && (lostTarget || turnPID.atGoal());
    }
}
