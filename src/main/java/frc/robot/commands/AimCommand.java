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
    private static final double MIN_DURATION = 0.5;

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

        this.timer = new Timer();

        Constants.tab_subsystems.addBoolean("Turn PID At", () -> this.turnPID.atGoal());
        Constants.tab_subsystems.addNumber("Turn PID Error", () -> {
            try {
                return this.turnPID.getPositionError();
            } catch (Exception e) {
                return 0;
            }
        });

        addRequirements(drivebase);
    }

    @Override
    public void initialize() {
        camera.setDriverMode(false);
        camera.setLED(VisionLEDMode.kOn);
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        PhotonPipelineResult result = camera.getLatestResult();

        if (result.hasTargets()) {
            Logger.l("Meters to target: " + CameraCalc.getDistanceMeters(camera));

            double calculatedValue = 4*turnPID.calculate(Units.degreesToRadians(result.getBestTarget().getYaw()));
            
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
    }

    @Override
    public void end(boolean interrupted) {
        camera.setDriverMode(true);
        camera.setLED(Constants.DEFAULT_LED_MODE);
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(MIN_DURATION) && turnPID.atGoal();
    }
}
