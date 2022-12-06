// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.common.hardware.VisionLEDMode;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.team5431.titan.core.leds.BlinkinPattern;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final int ID_FEEDER_BOTTOM = 15;
    public static final int ID_FEEDER_TOP = 16;
    public static final int ID_SHOOTER_LEFT = 19;
    public static final int ID_SHOOTER_RIGHT = 20;
    public static final int ID_INTAKE_LEFT = 22;
    public static final int ID_INTAKE_RIGHT = 23;
    public static final int ID_PIVOT_LEFT = 18;
    public static final int ID_PIVOT_RIGHT = 17;
    public static final int ID_CLIMBER_EXTEND = 14;
    public static final int ID_CLIMBER_HINGE = 13;
    public static final int ID_PIGEON2 = 21;

    public static final String CANBUS_DRIVETRAIN = "";
    public static final String CANBUS_SUBSYSTEM = "";

    public static final int SLOT_ANGLER = 5;
    public static final int SLOT_FEEDER_SENSOR = 1;
    public static final String CAMERA_NAME = "gloworm";
    public static final int SLOT_LEDS = 0;

    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.565; 
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.565;

    public static final double ROBOT_MASS_KG = Units.lbsToKilograms(110);

    // public static final int DRIVETRAIN_PIGEON_ID = 0;

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 2;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 1;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 32;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(80.244); // 85.430

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 8;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 7;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 12;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(139.482); // 140.449

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 4;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 3;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 10;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(50.098); // 49.307

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 6;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 5;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 9;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(27.598); // 30.234

    public static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(38.9);
    public static final double TARGET_HEIGHT_METERS = Units.feetToMeters(8) + Units.inchesToMeters(8);
    public static final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(27.3); // 29

    public static final VisionLEDMode DEFAULT_LED_MODE = VisionLEDMode.kOn;
    public static final boolean DRIVER_MODE = false;

    public static final int VISION_PIPELINE_INDEX = 0;

    public static final BlinkinPattern LEDPATTERN_DEFAULT = BlinkinPattern.YELLOW;
    public static final BlinkinPattern LEDPATTERN_SHOOT = BlinkinPattern.BLUE;
    public static final BlinkinPattern LEDPATTERN_SHOOT_BB = BlinkinPattern.GREEN;
    public static final BlinkinPattern LEDPATTERN_CLIMB = BlinkinPattern.STROBE_RED;
    public static final BlinkinPattern LEDPATTERN_AIM = BlinkinPattern.TWINKLES_RAINBOW_PALETTE;

    public static final ShuffleboardTab tab_subsystems = Shuffleboard.getTab("Subsystems");
    public static final ShuffleboardTab tab_commands = Shuffleboard.getTab("Commands");
}
