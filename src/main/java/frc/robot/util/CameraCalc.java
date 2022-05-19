package frc.robot.util;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.subsystems.Angler;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Shooter;
import frc.team5431.titan.core.misc.Logger;

public class CameraCalc {
    // Cache distance if target is lost
    // Also, use for holding calculated distance when locked to hub
    private static double cachedDistance = 8; // -1
    
    private static final ProfiledPIDController turnPID = new ProfiledPIDController(
            1, 
            0, 
            0.01, 
            new Constraints(
                    Drivebase.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 
                    Drivebase.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
            )
    );
    private static final double MIN_POWER = 0.1;

    static {
        turnPID.setGoal(0);
        turnPID.setTolerance(0.05, 0.2);
    }

    public static double getDistanceMeters(PhotonCamera camera) {
        return getDistanceMeters(camera, true);
    }

    public static double getDistanceMeters(PhotonCamera camera, boolean useLockedToHubDistance) {
        PhotonPipelineResult result = camera.getLatestResult();

        if (useLockedToHubDistance && Drivebase.lockedToHub) {
            return cachedDistance;
        }

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

    public static double getYawDegrees(PhotonCamera camera) {
        PhotonPipelineResult result = camera.getLatestResult();

        if (result.hasTargets()) {
            return result.getBestTarget().getYaw();
        }

        return 0;
    }

    public static double getYawRadians(PhotonCamera camera) {
        return Units.degreesToRadians(getYawDegrees(camera));
    }

    public static double getRotationToHub(PhotonCamera camera, double drivebaseVelocity, double angleOffsetFromGoingToHubRadians) {
        double distance = getDistanceMeters(camera, false);
        PhotonPipelineResult result = camera.getLatestResult();

        if (result.hasTargets()) {
            Logger.l("Meters to target: " + distance);

            double yaw = result.getBestTarget().getYaw();

            double distanceToDesired = Math.sqrt(
                drivebaseVelocity * drivebaseVelocity
              + distance * distance
              - 2 * drivebaseVelocity * distance * Math.cos(angleOffsetFromGoingToHubRadians)
            );

            if (Drivebase.lockedToHub) {
                cachedDistance = distanceToDesired;
            }

            double yawOffset = Units.radiansToDegrees(Math.asin(
                drivebaseVelocity
              * Math.sin(angleOffsetFromGoingToHubRadians)
              / distanceToDesired
            ));

            yaw -= yawOffset;

            double yawToTargetRadians = Units.degreesToRadians(yaw);

            double calculatedValue = 12*turnPID.calculate(yawToTargetRadians);
            // double calculatedValue = 4*yawToTargetRadians;

            // Logger.l("Aim calc: %s -> %s", yawToTargetRadians, calculatedValue);
            // Logger.l("Turn PID State: %s", turnPID.getPositionError());

            if (Math.abs(calculatedValue) <= MIN_POWER) {
                return 0;
            }

            return Math.copySign(
                Math.max(
                    Math.abs(calculatedValue),
                    Drivebase.MIN_ANGULAR_VELOCITY
                ), 
                calculatedValue
            );
        } else {
            return 0;
        }
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

    /** Equation:
     * y = logarithm(distance, 1.83452, 0.595659, 0.255365) / 
     *     quadratic(distance, 0.0160751, -0.104268, 2.58818)
     * (max with the MIN_LIMIT)
     */
    public static double calculateAngler(PhotonCamera camera) {
        double distance = getDistanceMeters(camera);

        if (distance < 0)
            return -1;
        
        double value = logarithm(distance, 1.83452, 0.595659, 0.255365) / 
                       quadratic(distance, 0.0160751, -0.104268, 2.58818);

        return MathUtil.clamp(value, Angler.DOWN_LIMIT, Angler.UP_LIMIT);
        // return MathUtil.clamp(Calc.map(distance, 2.15, 7.5, 0.45, 0.6), 0.45, 0.6); // prev max: 0.45/0.6
    }

    /** Equation:
     * y = 12000 * (quadratic(distance, 0.130406, -0.275359, 3.04742) /
     *              quadratic(distance, 0.0651734, 0.0764011, 2.52512))
     */
    public static double calculateRPM(PhotonCamera camera) {
        double distance = getDistanceMeters(camera);

        Logger.l("Distance: %s", distance);

        if (distance < 0) {
            return Shooter.VELOCITY_NORMAL;
        }

        return 12000 * (quadratic(distance, 0.130406, -0.275359, 3.04742) /
                        quadratic(distance, 0.0651734, 0.0764011, 2.52512)) * 0.97; // - 700;
        // return MathUtil.clamp(Calc.map(distance, 2.15, 7.45, 11500, 17750), 11300, 19000); // // prev max: 17750
        // return MathUtil.clamp(Calc.map(distance, 2.15, 7.45, 10000, 15050), 9000, 16000);
    }

    /**
     * Helper function for quadratic equations
     * 
     * ax^2 + bx + c
     */
    public static double quadratic(double x, double a, double b, double c) {
        return a * x * x
             + b * x
             + c;
    }

    /**
     * Helper function for logarithm equations
     * 
     * a * log(x - c) + b
     */
    public static double logarithm(double x, double a, double b, double c) {
        return a * Math.log10(x - c) + b; 
    }
}
