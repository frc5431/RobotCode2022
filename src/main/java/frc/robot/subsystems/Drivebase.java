// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.*;

import java.util.List;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.MkModuleConfiguration;
import com.swervedrivespecialties.swervelib.MkSwerveModuleBuilder;
import com.swervedrivespecialties.swervelib.MotorType;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import org.apache.commons.lang3.tuple.Triple;
import org.photonvision.PhotonCamera;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.CameraCalc;
import frc.team5431.titan.core.misc.Logger;

public class Drivebase extends SubsystemBase {
    public static enum GyroType {
        PIGEON2,
        NAVX,
        ANALOG_DEVICES;
    }

    public final GyroType CURRENT_GYRO_TYPE = GyroType.PIGEON2;

    /**
     * The maximum voltage that will be delivered to the drive motors.
     * <p>
     * This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial testing of the robot.
     */
    public static final double MAX_VOLTAGE = 12.0;
    //  Measure the drivetrain's maximum velocity or calculate the theoretical.
    //  The formula for calculating the theoretical maximum velocity is:
    //   <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> * pi
    //  By default this value is setup for a Mk3 standard module using Falcon500s to drive.
    //  An example of this constant for a Mk4 L2 module with NEOs to drive is:
    //   5880.0 / 60.0 * SdsModuleConfigurations.MK4_L2.getDriveReduction() * SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
    /**
     * The maximum velocity of the robot in meters per second.
     * <p>
     * This is a measure of how fast the robot should be able to drive in a straight line.
     * 
     * 
     * MK4_L2 = 4.96823045476
     */
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
                    SdsModuleConfigurations.MK4_L2.getDriveReduction() *
                    SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI;
    /**
     * The maximum angular velocity of the robot in radians per second.
     * <p>
     * This is a measure of how fast the robot can rotate in place.
     */
    // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
                    Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

    public static final double MIN_ANGULAR_VELOCITY = 0.6;
    // Max input acceleration (ChassisSpeeds meters per second per second) for x/y movement
    public static final double SLEW_RATE_LIMIT_TRANSLATION = MAX_VELOCITY_METERS_PER_SECOND * 2;
    // Max input acceleration (ChassisSpeeds radians per second per second) for rotational movement
    public static final double SLEW_RATE_LIMIT_ROTATION = MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * 10;

    public final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
                    // Front left
                    new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
                    // Front right
                    new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
                    // Back left
                    new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
                    // Back right
                    new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
    );

    // By default we use a Pigeon for our gyroscope. But if you use another gyroscope, like a NavX, you can change this.
    // The important thing about how you configure your gyroscope is that rotating the robot counter-clockwise should
    // cause the angle reading to increase until it wraps back over to zero.

    // NavX
    public final AHRS m_navx; // NavX connected over MXP
    // Analog Devices Gyro
    public final ADXRS450_Gyro m_adxr;
    // Pigeon 2.0
    public final WPI_Pigeon2 m_pigeon2;

    public final SwerveDrivePoseEstimator m_odometry;

    // These are our modules. We initialize them in the constructor.
    private final SwerveModule m_frontLeftModule;
    private final SwerveModule m_frontRightModule;
    private final SwerveModule m_backLeftModule;
    private final SwerveModule m_backRightModule;

    private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
    private Triple<Double, Double, Boolean> relativeDriving = null;

    private final SlewRateLimiter filter_vx;
    private final SlewRateLimiter filter_vy;
    private final SlewRateLimiter filter_or;

    public static boolean lockedToHub = false;
    private PhotonCamera camera;

    // private ShuffleboardTab tab;
    public final Field2d field2d;

    public Drivebase(PhotonCamera camera) {
        switch (CURRENT_GYRO_TYPE) {
            case ANALOG_DEVICES:
                m_adxr = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);
                m_navx = null;
                m_pigeon2 = null;
                break;
            case NAVX:
                m_adxr = null;
                m_navx = new AHRS(I2C.Port.kMXP, (byte) 200);
                m_pigeon2 = null;
                break;
            case PIGEON2:
                m_adxr = null;
                m_navx = null;
                m_pigeon2 = new WPI_Pigeon2(Constants.ID_PIGEON2, CANBUS_DRIVETRAIN);
                break;
            default:
                m_adxr = null;
                m_navx = null;
                m_pigeon2 = null;
        }

        // tab = Shuffleboard.getTab("Drivetrain");
        MkModuleConfiguration moduleConfig = MkModuleConfiguration.getDefaultSteerFalcon500();
        moduleConfig.setDriveCurrentLimit(40.0);
        moduleConfig.setSteerCurrentLimit(30.0);

        // There are 4 methods you can call to create your swerve modules.
        // The method you use depends on what motors you are using.
        //
        // Mk3SwerveModuleHelper.createFalcon500(...)
        //   Your module has two Falcon 500s on it. One for steering and one for driving.
        //
        // Mk3SwerveModuleHelper.createNeo(...)
        //   Your module has two NEOs on it. One for steering and one for driving.
        //
        // Mk3SwerveModuleHelper.createFalcon500Neo(...)
        //   Your module has a Falcon 500 and a NEO on it. The Falcon 500 is for driving and the NEO is for steering.
        //
        // Mk3SwerveModuleHelper.createNeoFalcon500(...)
        //   Your module has a NEO and a Falcon 500 on it. The NEO is for driving and the Falcon 500 is for steering.
        //
        // Similar helpers also exist for Mk4 modules using the Mk4SwerveModuleHelper class.

        // By default we will use Falcon 500s in standard configuration. But if you use a different configuration or motors
        // you MUST change it. If you do not, your code will crash on startup.
        // m_frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(
        //                 // This parameter is optional, but will allow you to see the current state of the module on the dashboard.
        //                 getSMLayout(tab.getLayout("Front Left Module", BuiltInLayouts.kList))
        //                                 .withPosition(0, 0),
        //                 moduleConfig,
        //                 // This can either be STANDARD or FAST depending on your gear configuration
        //                 Mk4SwerveModuleHelper.GearRatio.L2,
        //                 // This is the ID of the drive motor
        //                 FRONT_LEFT_MODULE_DRIVE_MOTOR,
        //                 // This is the ID of the steer motor
        //                 FRONT_LEFT_MODULE_STEER_MOTOR,
        //                 // This is the ID of the steer encoder
        //                 FRONT_LEFT_MODULE_STEER_ENCODER,
        //                 // This is how much the steer encoder is offset from true zero (In our case, zero is facing straight forward)
        //                 FRONT_LEFT_MODULE_STEER_OFFSET
        // );

        m_frontLeftModule = new MkSwerveModuleBuilder(moduleConfig)
                // .withLayout(getSMLayout(tab.getLayout("Front Left Module", BuiltInLayouts.kList))
                //         .withPosition(0, 0))
                .withGearRatio(SdsModuleConfigurations.MK4_L2)
                .withDriveMotor(MotorType.FALCON, FRONT_LEFT_MODULE_DRIVE_MOTOR, CANBUS_DRIVETRAIN)
                .withSteerMotor(MotorType.FALCON, FRONT_LEFT_MODULE_STEER_MOTOR, CANBUS_DRIVETRAIN)
                .withSteerEncoderPort(FRONT_LEFT_MODULE_STEER_ENCODER, CANBUS_DRIVETRAIN)
                .withSteerOffset(FRONT_LEFT_MODULE_STEER_OFFSET)
                .build();

        // We will do the same for the other modules
        m_frontRightModule = new MkSwerveModuleBuilder(moduleConfig)
                // .withLayout(getSMLayout(tab.getLayout("Front Right Module", BuiltInLayouts.kList))
                //         .withPosition(3, 0))
                .withGearRatio(SdsModuleConfigurations.MK4_L2)
                .withDriveMotor(MotorType.FALCON, FRONT_RIGHT_MODULE_DRIVE_MOTOR, CANBUS_DRIVETRAIN)
                .withSteerMotor(MotorType.FALCON, FRONT_RIGHT_MODULE_STEER_MOTOR, CANBUS_DRIVETRAIN)
                .withSteerEncoderPort(FRONT_RIGHT_MODULE_STEER_ENCODER, CANBUS_DRIVETRAIN)
                .withSteerOffset(FRONT_RIGHT_MODULE_STEER_OFFSET)
                .build();

        m_backLeftModule = new MkSwerveModuleBuilder(moduleConfig)
                // .withLayout(getSMLayout(tab.getLayout("Back Left Module", BuiltInLayouts.kList))
                //         .withPosition(6, 0))
                .withGearRatio(SdsModuleConfigurations.MK4_L2)
                .withDriveMotor(MotorType.FALCON, BACK_LEFT_MODULE_DRIVE_MOTOR, CANBUS_DRIVETRAIN)
                .withSteerMotor(MotorType.FALCON, BACK_LEFT_MODULE_STEER_MOTOR, CANBUS_DRIVETRAIN)
                .withSteerEncoderPort(BACK_LEFT_MODULE_STEER_ENCODER, CANBUS_DRIVETRAIN)
                .withSteerOffset(BACK_LEFT_MODULE_STEER_OFFSET)
                .build();

        m_backRightModule = new MkSwerveModuleBuilder(moduleConfig)
                // .withLayout(getSMLayout(tab.getLayout("Back Right Module", BuiltInLayouts.kList))
                //         .withPosition(9, 0))
                .withGearRatio(SdsModuleConfigurations.MK4_L2)
                .withDriveMotor(MotorType.FALCON, BACK_RIGHT_MODULE_DRIVE_MOTOR, CANBUS_DRIVETRAIN)
                .withSteerMotor(MotorType.FALCON, BACK_RIGHT_MODULE_STEER_MOTOR, CANBUS_DRIVETRAIN)
                .withSteerEncoderPort(BACK_RIGHT_MODULE_STEER_ENCODER, CANBUS_DRIVETRAIN)
                .withSteerOffset(BACK_RIGHT_MODULE_STEER_OFFSET)
                .build();

        m_odometry = new SwerveDrivePoseEstimator(m_kinematics, getGyroscopeRotation(), getPositions(), new Pose2d());

        // ((WPI_TalonFX) m_frontLeftModule.getDriveMotor()).configOpenloopRamp(RAMPING_FROM_0_TO_FULL);
        // ((WPI_TalonFX) m_frontRightModule.getDriveMotor()).configOpenloopRamp(RAMPING_FROM_0_TO_FULL);
        // ((WPI_TalonFX) m_backLeftModule.getDriveMotor()).configOpenloopRamp(RAMPING_FROM_0_TO_FULL);
        // ((WPI_TalonFX) m_backRightModule.getDriveMotor()).configOpenloopRamp(RAMPING_FROM_0_TO_FULL);

        filter_vx = new SlewRateLimiter(SLEW_RATE_LIMIT_TRANSLATION);
        filter_vy = new SlewRateLimiter(SLEW_RATE_LIMIT_TRANSLATION);
        filter_or = new SlewRateLimiter(SLEW_RATE_LIMIT_ROTATION);

        ShuffleboardLayout chassisSpeedsLayout = Constants.tab_subsystems.getLayout("ChassisSpeeds", BuiltInLayouts.kList)
                .withSize(2, 3)
                .withPosition(21, 0);
        chassisSpeedsLayout.addNumber("vX", () -> m_chassisSpeeds.vxMetersPerSecond);
        chassisSpeedsLayout.addNumber("vY", () -> m_chassisSpeeds.vyMetersPerSecond);
        chassisSpeedsLayout.addNumber("oR", () -> m_chassisSpeeds.omegaRadiansPerSecond);

        this.camera = camera;

        field2d = new Field2d();

        Constants.tab_subsystems.add("Field", field2d)
                .withWidget(BuiltInWidgets.kField)
                .withPosition(8, 0)
                .withSize(6, 4);

        Constants.tab_subsystems.addNumber("Gyro Rotation", () -> this.getGyroscopeRotation().getDegrees())
                .withPosition(15, 3)
                .withSize(2, 1);

        Constants.tab_subsystems.addNumber("Gyro Pitch", () -> this.getGyro().getPitch())
                .withPosition(21, 3)
                .withSize(2, 1);

        Constants.tab_subsystems.addNumber("Gyro Roll", () -> this.getGyro().getRoll())
                .withPosition(21, 4)
                .withSize(2, 1);

        Constants.tab_subsystems.addBoolean("Locked to Hub", () -> Drivebase.lockedToHub)
                .withPosition(4, 1)
                .withSize(2, 1);
    }

    // private ShuffleboardLayout getSMLayout(ShuffleboardLayout layout) {
    //     return layout.withSize(3, 5);
    // }

    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(getGyroscopeRotation(), getPositions(), pose);
    }

    /**
     * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
     * 'forwards' direction.
     */
    public void zeroGyroscope() {
        Logger.l("Zeroing gyro!");

        // Pigeon
        // m_pigeon.setFusedHeading(0.0);

        switch (CURRENT_GYRO_TYPE) {
            case ANALOG_DEVICES:
                m_adxr.reset();
                break;
            case NAVX:
                m_navx.zeroYaw();
                break;
            case PIGEON2:
                m_pigeon2.reset();
                break;
        }
    }

    public void resetGyroAt(double yaw) {
        m_pigeon2.setYaw(yaw);
    }

    // @Log(name = "Gyroscope Rot", tabName = "Subsystems")
    public Rotation2d getGyroscopeRotation() {
        // Pigeon
        // return Rotation2d.fromDegrees(m_pigeon.getFusedHeading());

        // NavX
        // if (m_navx.isMagnetometerCalibrated()) {
        //     // We will only get valid fused headings if the magnetometer is calibrated
        //     return Rotation2d.fromDegrees(m_navx.getFusedHeading());
        // }

        // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes the angle increase.
        // return Rotation2d.fromDegrees(360.0 - m_navx.getYaw());

        // Analog Devices
        // return m_adxr.getRotation2d();

        // Pigeon 2.0
        return m_pigeon2.getRotation2d();
    }

    public void driveController(ChassisSpeeds chassisSpeeds) {
        ChassisSpeeds speedsModified = new ChassisSpeeds(
            filter_vx.calculate(chassisSpeeds.vxMetersPerSecond),
            filter_vy.calculate(chassisSpeeds.vyMetersPerSecond),
            filter_or.calculate(chassisSpeeds.omegaRadiansPerSecond)
        );
        driveRaw(speedsModified);
    }

    public void driveRaw(ChassisSpeeds chassisSpeeds) {
        m_chassisSpeeds = chassisSpeeds;
        relativeDriving = null;
    }

    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        m_odometry.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
    }

    public void driveRawTranslate(double x, double y) {
        m_chassisSpeeds.vxMetersPerSecond = x;
        m_chassisSpeeds.vyMetersPerSecond = y;
        relativeDriving = null;
    }

    public void driveRawRotate(double rad) {
        m_chassisSpeeds.omegaRadiansPerSecond = rad;
        relativeDriving = null;
    }

    public void stop() {
        driveRaw(new ChassisSpeeds());
    }

    public void driveRelative(double drive, double turn) {
        driveRelative(drive, turn, false);
    }

    public void driveRelative(double drive, double turn, boolean curve) {
        relativeDriving = Triple.of(drive, turn, curve);
    }

    public SwerveModuleState[] getStates() {
        return new SwerveModuleState[] {
            m_frontLeftModule.getState(),
            m_frontRightModule.getState(),
            m_backLeftModule.getState(),
            m_backRightModule.getState()
        };
    }

    public SwerveModulePosition[] getPositions() {
        return new SwerveModulePosition[] {
            m_frontLeftModule.getPosition(),
            m_frontRightModule.getPosition(),
            m_backLeftModule.getPosition(),
            m_backRightModule.getPosition()
        };
    }

    @Override
    public void periodic() {
        m_odometry.update(getGyroscopeRotation(), getPositions());
        if (camera.getLatestResult().hasTargets()) {
            double distanceMeters = CameraCalc.getDistanceMeters(camera);
            double xOffset = CameraCalc.getYawDegrees(camera);
            double x = 8.23 - (distanceMeters * 
                Math.cos(Math.toRadians(getGyroscopeRotation().getDegrees() + 180 
                - xOffset)));
            double y = 4.165 - (distanceMeters * 
                Math.sin(Math.toRadians(getGyroscopeRotation().getDegrees() + 180
                - xOffset)));//plus or minus xoffset???
        
            m_odometry.resetPosition(getGyroscopeRotation(), getPositions(), new Pose2d(x, y, getGyroscopeRotation()));
        }
        field2d.setRobotPose(m_odometry.getEstimatedPosition());

        // Hockey-lock by setting rotation to realllly low number
        if (Math.abs(m_chassisSpeeds.vxMetersPerSecond) < 0.1) m_chassisSpeeds.vxMetersPerSecond = 0;
        if (Math.abs(m_chassisSpeeds.vyMetersPerSecond) < 0.1) m_chassisSpeeds.vyMetersPerSecond = 0;
        if (m_chassisSpeeds.vxMetersPerSecond == 0 && m_chassisSpeeds.vyMetersPerSecond == 0 && m_chassisSpeeds.omegaRadiansPerSecond == 0) {
            m_chassisSpeeds.omegaRadiansPerSecond = 0.00001;
        }

        if (lockedToHub) {
            double velocity = Math.sqrt(
                m_chassisSpeeds.vxMetersPerSecond * m_chassisSpeeds.vxMetersPerSecond +
                m_chassisSpeeds.vyMetersPerSecond * m_chassisSpeeds.vyMetersPerSecond
            );
            m_chassisSpeeds.omegaRadiansPerSecond = CameraCalc.getRotationToHub(
                camera, 
                velocity, // TODO: test modifiers to velocity for units
                Math.atan2(m_chassisSpeeds.vyMetersPerSecond, -m_chassisSpeeds.vxMetersPerSecond)
                    - CameraCalc.getYawRadians(camera)
            );
        }

        SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);

        if (relativeDriving != null) {
            WheelSpeeds diffSpeeds;
            if (relativeDriving.getRight()) {
                diffSpeeds = DifferentialDrive.curvatureDriveIK(relativeDriving.getLeft(), relativeDriving.getMiddle(), false);
            } else {
                diffSpeeds = DifferentialDrive.arcadeDriveIK(relativeDriving.getLeft(), relativeDriving.getMiddle(), false);
            }

            Logger.l("Relative driving! Left: %s - Right: %s", diffSpeeds.left, diffSpeeds.right);

            states = new SwerveModuleState[] {
                new SwerveModuleState(MAX_VELOCITY_METERS_PER_SECOND * diffSpeeds.left, getGyroscopeRotation()), // FL
                new SwerveModuleState(MAX_VELOCITY_METERS_PER_SECOND * diffSpeeds.right, getGyroscopeRotation()), // FR
                new SwerveModuleState(MAX_VELOCITY_METERS_PER_SECOND * diffSpeeds.left, getGyroscopeRotation()), // BL
                new SwerveModuleState(MAX_VELOCITY_METERS_PER_SECOND * diffSpeeds.right, getGyroscopeRotation())  // BR
            };
        }

        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);
        // SwerveModuleState.optimize(desiredState, currentAngle) maybe?

        double flVoltage;
        double frVoltage;
        double blVoltage;
        double brVoltage;

        flVoltage = states[0].speedMetersPerSecond;
        frVoltage = states[1].speedMetersPerSecond;
        blVoltage = states[2].speedMetersPerSecond;
        brVoltage = states[3].speedMetersPerSecond;

        // flVoltage = MathUtil.clamp(flVoltage, 0, MAX_VELOCITY_METERS_PER_SECOND);
        // frVoltage = MathUtil.clamp(frVoltage, 0, MAX_VELOCITY_METERS_PER_SECOND);
        // blVoltage = MathUtil.clamp(blVoltage, 0, MAX_VELOCITY_METERS_PER_SECOND);
        // brVoltage = MathUtil.clamp(brVoltage, 0, MAX_VELOCITY_METERS_PER_SECOND);

        // SmartDashboard.putNumber("Front Left Velocity", flVoltage);
        // SmartDashboard.putNumber("Front Right Velocity", frVoltage);
        // SmartDashboard.putNumber("Back Left Velocity", blVoltage);
        // SmartDashboard.putNumber("Back Right Velocity", brVoltage);

        flVoltage = flVoltage / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE;
        frVoltage = frVoltage / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE;
        blVoltage = blVoltage / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE;
        brVoltage = brVoltage / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE;

        m_frontLeftModule.set(flVoltage, states[0].angle.getRadians());
        m_frontRightModule.set(frVoltage, states[1].angle.getRadians());
        m_backLeftModule.set(blVoltage, states[2].angle.getRadians());
        m_backRightModule.set(brVoltage, states[3].angle.getRadians());
    }

    public Command resetYawCommand(double yaw) {
        return Commands.runOnce(() -> this.resetGyroAt(yaw));
    }

    public void setNeutralModeDrive(NeutralMode nm) {
        getSwerveModules().forEach((module) -> {
            ((WPI_TalonFX) module.getDriveMotor()).setNeutralMode(nm);
        });
    }

    public void setNeutralModeSteer(NeutralMode nm) {
        getSwerveModules().forEach((module) -> {
            ((WPI_TalonFX) module.getSteerMotor()).setNeutralMode(nm);
        });
    }

    public WPI_Pigeon2 getGyro() {
        return m_pigeon2;
    }

    public List<SwerveModule> getSwerveModules() {
        return List.of(m_frontLeftModule, m_frontRightModule, m_backLeftModule, m_backRightModule);
    }
}
