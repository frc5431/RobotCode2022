// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

import frc.robot.util.MotionMagic;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final int ID_FEEDER_BOTTOM = 13;
    public static final int ID_FEEDER_TOP = 14;
    public static final int ID_SHOOTER_LEFT = -1;
    public static final int ID_SHOOTER_RIGHT = -1;
    public static final int ID_INTAKE = -1;

    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.495;
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.495;

    // public static final int DRIVETRAIN_PIGEON_ID = 0;

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 8;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 7;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 12;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(261.299);

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 6;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 5;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 11;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(112.852);

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 2;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 1;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 9;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(-203.291);

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 4;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 3;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 10;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(53.789);

    public static final NeutralMode PIVOT_NEUTRALMODE = null;
    public static final TalonFXInvertType PIVOT_REVERSE = null;
    public static final double PIVOT_UP_LIMIT = 0;
    public static final double PIVOT_DOWN_LIMIT = 0;
    public static final int DRIVEBASE_TIMEOUT_MS = 0;
    public static final double PIVOT_DEFAULT_SPEED = 0;
    public static final MotionMagic PIVOT_MOTION_MAGIC = null;
    public static final double PIVOT_COSINE_MULT = 0;
    public static final double PIVOT_AFFECT_GRAVITY = 0;
    public static final double PIVOT_ERROR_RANGE = 0;
    public static final InvertType SHOOTER_FLYWHEEL_REVERSE = null;






}
