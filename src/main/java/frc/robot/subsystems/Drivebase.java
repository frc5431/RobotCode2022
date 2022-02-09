// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team5431.titan.swerve.Mk4ModuleConfigurationExt;
import frc.team5431.titan.swerve.Mk4SwerveModuleHelperExt;

public class Drivebase extends SubsystemBase {
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
    // Pigeon
    // private final PigeonIMU m_pigeon = new PigeonIMU(DRIVETRAIN_PIGEON_ID);
    // NavX
    public final AHRS m_navx = new AHRS(I2C.Port.kMXP, (byte) 200); // NavX connected over MXP
    // Analog Devices Gyro
    // public final ADXRS450_Gyro m_adxr = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);

    public final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, getGyroscopeRotation());

    // These are our modules. We initialize them in the constructor.
    private final SwerveModule m_frontLeftModule;
    private final SwerveModule m_frontRightModule;
    private final SwerveModule m_backLeftModule;
    private final SwerveModule m_backRightModule;

    private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    private ShuffleboardTab tab;

    public Drivebase() {
        tab = Shuffleboard.getTab("Drivetrain");
        Mk4ModuleConfigurationExt moduleConfig = Mk4ModuleConfigurationExt.getDefaultFalcon500();

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
        // Similar helpers also exist for Mk4 modules using the Mk4SwerveModuleHelperExt class.

        // By default we will use Falcon 500s in standard configuration. But if you use a different configuration or motors
        // you MUST change it. If you do not, your code will crash on startup.
        m_frontLeftModule = Mk4SwerveModuleHelperExt.createFalcon500(
                        // This parameter is optional, but will allow you to see the current state of the module on the dashboard.
                        tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                                        .withSize(2, 3)
                                        .withPosition(0, 0),
                        moduleConfig,
                        // This can either be STANDARD or FAST depending on your gear configuration
                        Mk4SwerveModuleHelperExt.GearRatio.L2,
                        // This is the ID of the drive motor
                        FRONT_LEFT_MODULE_DRIVE_MOTOR,
                        // This is the ID of the steer motor
                        FRONT_LEFT_MODULE_STEER_MOTOR,
                        // This is the ID of the steer encoder
                        FRONT_LEFT_MODULE_STEER_ENCODER,
                        // This is how much the steer encoder is offset from true zero (In our case, zero is facing straight forward)
                        FRONT_LEFT_MODULE_STEER_OFFSET
        );

        // We will do the same for the other modules
        m_frontRightModule = Mk4SwerveModuleHelperExt.createFalcon500(
                        tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                                        .withSize(2, 3)
                                        .withPosition(2, 0),
                        moduleConfig,
                        Mk4SwerveModuleHelperExt.GearRatio.L2,
                        FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                        FRONT_RIGHT_MODULE_STEER_MOTOR,
                        FRONT_RIGHT_MODULE_STEER_ENCODER,
                        FRONT_RIGHT_MODULE_STEER_OFFSET
        );

        m_backLeftModule = Mk4SwerveModuleHelperExt.createFalcon500(
                        tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                                        .withSize(2, 3)
                                        .withPosition(4, 0),
                        moduleConfig,
                        Mk4SwerveModuleHelperExt.GearRatio.L2,
                        BACK_LEFT_MODULE_DRIVE_MOTOR,
                        BACK_LEFT_MODULE_STEER_MOTOR,
                        BACK_LEFT_MODULE_STEER_ENCODER,
                        BACK_LEFT_MODULE_STEER_OFFSET
        );

        m_backRightModule = Mk4SwerveModuleHelperExt.createFalcon500(
                        tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                                        .withSize(2, 3)
                                        .withPosition(6, 0),
                        moduleConfig,
                        Mk4SwerveModuleHelperExt.GearRatio.L2,
                        BACK_RIGHT_MODULE_DRIVE_MOTOR,
                        BACK_RIGHT_MODULE_STEER_MOTOR,
                        BACK_RIGHT_MODULE_STEER_ENCODER,
                        BACK_RIGHT_MODULE_STEER_OFFSET
        );

        double secondsFromNeutralToFull = 0.5;

        ((TalonFX) m_frontLeftModule.getDriveMotor()).configOpenloopRamp(secondsFromNeutralToFull);
        ((TalonFX) m_frontRightModule.getDriveMotor()).configOpenloopRamp(secondsFromNeutralToFull);
        ((TalonFX) m_backLeftModule.getDriveMotor()).configOpenloopRamp(secondsFromNeutralToFull);
        ((TalonFX) m_backRightModule.getDriveMotor()).configOpenloopRamp(secondsFromNeutralToFull);

        ShuffleboardLayout chassisSpeedsLayout = tab.getLayout("ChassisSpeeds", BuiltInLayouts.kList)
                .withSize(3, 4)
                .withPosition(8, 0);
        chassisSpeedsLayout.addNumber("vX", () -> m_chassisSpeeds.vxMetersPerSecond);
        chassisSpeedsLayout.addNumber("vY", () -> m_chassisSpeeds.vyMetersPerSecond);
        chassisSpeedsLayout.addNumber("oR", () -> m_chassisSpeeds.omegaRadiansPerSecond);
    }

    /**
     * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
     * 'forwards' direction.
     */
    public void zeroGyroscope() {
        // Pigeon
        // m_pigeon.setFusedHeading(0.0);

        // NavX
        m_navx.zeroYaw();

        // Analog Devices
        // m_adxr.reset();
    }

    public Rotation2d getGyroscopeRotation() {
        // Pigeon
        // return Rotation2d.fromDegrees(m_pigeon.getFusedHeading());

        // NavX
        // if (m_navx.isMagnetometerCalibrated()) {
        //     // We will only get valid fused headings if the magnetometer is calibrated
        //     return Rotation2d.fromDegrees(m_navx.getFusedHeading());
        // }

        // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes the angle increase.
        return Rotation2d.fromDegrees(360.0 - m_navx.getYaw());

        // Analog Devices
        // return m_adxr.getRotation2d();
    }

    public void driveController(ChassisSpeeds chassisSpeeds) {
        driveRaw(chassisSpeeds);
    }

    public void driveRaw(ChassisSpeeds chassisSpeeds) {
        m_chassisSpeeds = chassisSpeeds;
    }

    public static SwerveModuleState getModuleState(SwerveModule module) {
        return new SwerveModuleState(module.getDriveVelocity(), Rotation2d.fromDegrees(module.getSteerAngle()));
    }

    public SwerveModuleState[] getStates() {
        return new SwerveModuleState[] {
            getModuleState(m_frontLeftModule),
            getModuleState(m_frontRightModule),
            getModuleState(m_backLeftModule),
            getModuleState(m_backRightModule)
        };
    }

    @Override
    public void periodic() {
        m_odometry.update(getGyroscopeRotation(), getStates());

        // Hockey-lock by setting rotation to realllly low number
        if (m_chassisSpeeds.omegaRadiansPerSecond == 0) {
            m_chassisSpeeds.omegaRadiansPerSecond = 0.00001;
        }

        SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
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

        flVoltage = MathUtil.clamp(flVoltage, 0, MAX_VELOCITY_METERS_PER_SECOND);
        frVoltage = MathUtil.clamp(frVoltage, 0, MAX_VELOCITY_METERS_PER_SECOND);
        blVoltage = MathUtil.clamp(blVoltage, 0, MAX_VELOCITY_METERS_PER_SECOND);
        brVoltage = MathUtil.clamp(brVoltage, 0, MAX_VELOCITY_METERS_PER_SECOND);

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
}
