package frc.robot.commands;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Systems;
import frc.robot.commands.subsystems.AnglerCommand;
import frc.robot.util.CameraCalc;

public class AimAndShootCommand extends ParallelCommandGroup {
    public static final double AIM_DELAY = 1.0; // delay after start to aim
    public static final double SHOOT_DELAY = 1.0; // delay after aim to shoot

    /*
     * 7.5m  - 17750 - 0.65
     * 2.15m - 11240 - 0.45
     */

    private final PhotonCamera camera;
    
    public AimAndShootCommand(Systems systems) {
        this.camera = systems.getCamera();

        addCommands(
            new AimCommand(systems),
            new WaitCommand(AIM_DELAY)
                .andThen(new AnglerCommand(systems, AnglerCommand.COMMAND.SET, () -> CameraCalc.calculateAngler(camera) )),
            new WaitCommand(AIM_DELAY + SHOOT_DELAY)
                .andThen(new ShootCommand(systems, () -> CameraCalc.calculateRPM(camera) ))
        );
    }
}
