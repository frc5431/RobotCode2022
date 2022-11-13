package frc.robot.commands;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Systems;
import frc.robot.util.CameraCalc;

public class ShootWithCalcRPMCommand extends ParallelCommandGroup {

    /*
     * 7.5m  - 17750 - 0.65
     * 2.15m - 11240 - 0.45
     */

    private final PhotonCamera camera;

    public ShootWithCalcRPMCommand(Systems systems) {
        this(systems, true);
    }
    
    public ShootWithCalcRPMCommand(Systems systems, boolean waitForFlywheel) {
        this.camera = systems.getCamera();

        addCommands(
            new TimedFeedAndShootCommand(systems, () -> CameraCalc.calculateRPM(camera), waitForFlywheel)
        );
    }
}
