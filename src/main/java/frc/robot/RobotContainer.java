// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.subsystems.Drivebase;
import frc.team5431.titan.core.joysticks.Xbox;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final Drivebase drivebase = new Drivebase();

    private final Xbox driver = new Xbox(0);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Set up the default command for the drivetrain.
        // The controls are for field-oriented driving:
        // Left stick Y axis -> forward and backwards movement
        // Left stick X axis -> left and right movement
        // Right stick X axis -> rotation
        drivebase.setDefaultCommand(new DefaultDriveCommand(
                        drivebase,
                        () -> modifyAxis(driver.getRawAxis(Xbox.Axis.LEFT_Y)) * Drivebase.MAX_VELOCITY_METERS_PER_SECOND,
                        () -> modifyAxis(driver.getRawAxis(Xbox.Axis.LEFT_X)) * Drivebase.MAX_VELOCITY_METERS_PER_SECOND,
                        () -> modifyAxis(driver.getRawAxis(Xbox.Axis.RIGHT_X)) * Drivebase.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
        ));

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // Back button zeros the gyroscope
        new JoystickButton(driver, Xbox.Button.Y.ordinal() + 1)
                        // No requirements because we don't need to interrupt anything
                        .whenPressed(drivebase::zeroGyroscope);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("TestSwerve", Drivebase.MAX_VELOCITY_METERS_PER_SECOND, 2.0);

        return new PPSwerveControllerCommand(
                trajectory, 
                () -> drivebase.m_odometry.getPoseMeters(), 
                drivebase.m_kinematics, 
                new PIDController(0.5, 0, 0.01), 
                new PIDController(0.5, 0, 0.01), 
                new ProfiledPIDController(0.5, 0, 0.01, new Constraints(Drivebase.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 1)),
                (states) -> drivebase.driveRaw(drivebase.m_kinematics.toChassisSpeeds(states)), 
                drivebase)
            .andThen(new InstantCommand(
                    () -> drivebase.driveRaw(new ChassisSpeeds())
                    , drivebase));
    }

    private static double deadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
            if (value > 0.0) {
                return (value - deadband) / (1.0 - deadband);
            } else {
                return (value + deadband) / (1.0 - deadband);
            }
        } else {
            return 0.0;
        }
    }

    private static double modifyAxis(double value) {
        // Deadband
        value = deadband(value, 0.08);

        // Square the axis
        value = Math.copySign(value * value, value);

        return value;
    }
}
