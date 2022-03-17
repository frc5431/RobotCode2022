package frc.robot.util;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import frc.team5431.titan.core.misc.Calc;

public class CameraCalc {
    public static double getDistanceMeters(PhotonCamera camera) {
        PhotonPipelineResult result = camera.getLatestResult();

        if (result.hasTargets()) {
            return PhotonUtils.calculateDistanceToTargetMeters(
                    Constants.CAMERA_HEIGHT_METERS, 
                    Constants.TARGET_HEIGHT_METERS, 
                    Constants.CAMERA_PITCH_RADIANS, 
                    Units.degreesToRadians(result.getBestTarget().getPitch())
            );
        }

        return -1;
    }

    public static double calculateAngler(PhotonCamera camera) {
        double distance = getDistanceMeters(camera);

        if (distance < 0)
            return -1;

        return MathUtil.clamp(Calc.map(distance, 2.15, 7.5, 0.45, 0.65), 0.45, 0.65);
    }

    public static double calculateRPM(PhotonCamera camera) {
        double distance = getDistanceMeters(camera);

        if (distance < 0)
            return Shooter.VELOCITY_NORMAL;
        
        return MathUtil.clamp(Calc.map(distance, 2.15, 7.5, 11240, 17750), 10700, 19000);
    }
}
