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
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.*;
import frc.robot.commands.subsystems.*;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Shooter;
import frc.team5431.titan.core.joysticks.LogitechExtreme3D;
import frc.team5431.titan.core.joysticks.Xbox;
import frc.team5431.titan.core.misc.Calc;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final Drivebase drivebase = new Drivebase();
    private final Systems systems = new Systems();

    private final Xbox driver = new Xbox(0);
    private final Joystick buttonBoard = new Joystick(1);
    private final LogitechExtreme3D operator = new LogitechExtreme3D(2);

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
                        () -> modifyAxis(-driver.getRawAxis(Xbox.Axis.LEFT_Y)) * Drivebase.MAX_VELOCITY_METERS_PER_SECOND,
                        () -> modifyAxis(-driver.getRawAxis(Xbox.Axis.LEFT_X)) * Drivebase.MAX_VELOCITY_METERS_PER_SECOND,
                        () -> modifyAxis(-driver.getRawAxis(Xbox.Axis.RIGHT_X)) * Drivebase.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
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
        // Y button zeros the gyroscope
        new JoystickButton(driver, Xbox.Button.Y.ordinal() + 1)
                // No requirements because we don't need to interrupt anything
                .whenPressed(drivebase::zeroGyroscope);
        
        // D-Pad cardinal directions
        new POVButton(driver, 0)
                .whileHeld(
                    () -> drivebase.driveController(new ChassisSpeeds(Drivebase.MAX_VELOCITY_METERS_PER_SECOND, 0, 0)), drivebase);
        new POVButton(driver, 180)
                .whileHeld(
                    () -> drivebase.driveController(new ChassisSpeeds(-Drivebase.MAX_VELOCITY_METERS_PER_SECOND, 0, 0)), drivebase);
        new POVButton(driver, 270)
                .whileHeld(
                    () -> drivebase.driveController(new ChassisSpeeds(0, Drivebase.MAX_VELOCITY_METERS_PER_SECOND, 0)), drivebase);
        new POVButton(driver, 90)
                .whileHeld(
                    () -> drivebase.driveController(new ChassisSpeeds(0, -Drivebase.MAX_VELOCITY_METERS_PER_SECOND, 0)), drivebase);
        
        // Intake (Manual)
        new JoystickButton(buttonBoard, 7)
                .whileHeld(new IntakeCommand(systems, false));
        
        // Intake Reverse (Manual)
        new JoystickButton(buttonBoard, 3)
                .whileHeld(new IntakeCommand(systems, true));
        
        // Pivot Up
        new JoystickButton(buttonBoard, 9)
                .whileHeld(() -> systems.getPivot().set(0.1), systems.getPivot())
                .whenReleased(() -> systems.getPivot().set(0), systems.getPivot());
        
        // Pivot Down
        new JoystickButton(buttonBoard, 3)
                .whileHeld(() -> systems.getPivot().set(-0.1), systems.getPivot())
                .whenReleased(() -> systems.getPivot().set(0), systems.getPivot());

        // Trigger/slider Shoot
        new JoystickButton(operator, LogitechExtreme3D.Button.TRIGGER.ordinal() + 1)
                .whileHeld(new ShooterCommand(systems, 
                        () -> Calc.map(
                                operator.getRawAxis(LogitechExtreme3D.Axis.SLIDER), 
                                        1.0, -1.0, 
                                        0, Shooter.MAX_VELOCITY)));
        
        // Floor Intake
        new JoystickButton(buttonBoard, 4)
                .toggleWhenPressed(new FloorIntakeCommand(systems));
        
        // Reject
        new JoystickButton(buttonBoard, 1)
                .whileHeld(new ShootCommand(systems, Shooter.Velocity.CLOSE));

        // Shoot 
        new JoystickButton(buttonBoard, 6)
                .whileHeld(new ShootCommand(systems, Shooter.Velocity.FAR));
        
        // // Shoot Close (Manual)
        // new JoystickButton(operator, LogitechExtreme3D.Button.SEVEN.ordinal() + 1)
        //         .whileHeld(new ShooterCommand(systems, Shooter.Velocity.CLOSE));

        // // Shoot Far (Manual)
        // new JoystickButton(operator, LogitechExtreme3D.Button.EIGHT.ordinal() + 1)
        //         .whileHeld(new ShooterCommand(systems, Shooter.Velocity.FAR));
        
        // Feed Both Up
        new POVButton(operator, 0)
                .whileHeld(new FeedEverything(systems, false));
        
        // Feed Both Down
        new POVButton(operator, 180)
                .whileHeld(new FeedEverything(systems, true));
        
        // Stop All
        new JoystickButton(buttonBoard, 12)
                .whileHeld(new StopAllCommand(systems));

        // new JoystickButton(operator, LogitechExtreme3D.Button.TWELVE.ordinal() + 1)
        //         .whileHeld(() -> systems.getShooter().set(Shooter.MAX_VELOCITY), systems.getShooter());
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
        value = deadband(value, 0.075);

        // Square the axis
        value = Math.copySign(value * value, value);

        return value;
    }
}
