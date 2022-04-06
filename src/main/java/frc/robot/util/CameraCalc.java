package frc.robot.util;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.subsystems.Angler;
import frc.robot.subsystems.Shooter;
import frc.team5431.titan.core.misc.Logger;

public class CameraCalc {
    private static double cachedDistance = 8; // -1

    public static double getDistanceMeters(PhotonCamera camera) {
        PhotonPipelineResult result = camera.getLatestResult();

        if (result.hasTargets()) {
            double calculated = PhotonUtils.calculateDistanceToTargetMeters(
                    Constants.CAMERA_HEIGHT_METERS, 
                    Constants.TARGET_HEIGHT_METERS, 
                    Constants.CAMERA_PITCH_RADIANS, 
                    Units.degreesToRadians(result.getBestTarget().getPitch())
            );
            cachedDistance = calculated;
            return calculated;
        }

        return cachedDistance;
    }

    /*
     * 1.30m - 0.25 - 12776
     * 1.50m - 0.325 - 12609
     * 2.0m - 0.4 - 12276
     * 2.5m - 0.525 - 12276
     * 3.0m - 0.6 - 12192
     * 3.5m - 0.625 - 12192
     * 4.0m - 0.65 - 12626
     * 4.5m - 0.725 - 12626
     * 5.0m - 0.75 - 13110
     * 5.5m - 0.775 - 13393
     * 6.0m - 0.775 - 13677
     * 6.5m - 
     * 8.0m - 0.8 - 15095
     */

    /* Equation:
     * y = 0.676876 * log (distance) + 0.195982
     * (max with the MIN_LIMIT)
     */
    public static double calculateAngler(PhotonCamera camera) {
        double distance = getDistanceMeters(camera);

        if (distance < 0)
            return -1;
        
        double value = 0.676876 * Math.log10(distance) + 0.195982;

        return MathUtil.clamp(value, Angler.DOWN_LIMIT, Angler.UP_LIMIT);
        // return MathUtil.clamp(Calc.map(distance, 2.15, 7.5, 0.45, 0.6), 0.45, 0.6); // prev max: 0.45/0.6
    }

    /* Equation:
     * y = 12000 ( (0.0317023 * (1.36189 ^ distance) ) + 0.973105)
     */
    public static double calculateRPM(PhotonCamera camera) {
        double distance = getDistanceMeters(camera);

        Logger.l("Distance: %s", distance);

        if (distance < 0) {
            return Shooter.VELOCITY_NORMAL;
        }

        return 12000 * ( 0.0317023 * Math.pow(1.36189, distance) + 0.973105 );
        // return MathUtil.clamp(Calc.map(distance, 2.15, 7.45, 11500, 17750), 11300, 19000); // // prev max: 17750
        // return MathUtil.clamp(Calc.map(distance, 2.15, 7.45, 10000, 15050), 9000, 16000);
    }
}
