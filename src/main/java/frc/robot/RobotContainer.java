// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.AimCommand;
import frc.robot.commands.AutonCommand;
import frc.robot.commands.FeedEverything;
import frc.robot.commands.FloorIntakeCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.StopAllCommand;
import frc.robot.commands.subsystems.AnglerCommand;
import frc.robot.commands.subsystems.ClimberExtendCommand;
import frc.robot.commands.subsystems.ClimberHingeCommand;
import frc.robot.commands.subsystems.DefaultDriveCommand;
import frc.robot.commands.subsystems.IntakeCommand;
import frc.robot.commands.subsystems.PivotCommand;
import frc.robot.commands.subsystems.ShooterCommand;
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
    private final Systems systems = new Systems();
    private final Drivebase drivebase = systems.getDrivebase();
    private final PhotonCamera camera = systems.getCamera();

    private final PowerDistribution pdh = new PowerDistribution();

    private final Xbox driver = new Xbox(0);
    private final Joystick buttonBoard = new Joystick(1);
    private final LogitechExtreme3D operator = new LogitechExtreme3D(2);

    private final SendableChooser<AutonCommand.State> autonChooser;

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

        camera.setLED(Constants.DEFAULT_LED_MODE);

        Constants.tab_subsystems.addNumber("PD Volts", pdh::getVoltage);
        Constants.tab_subsystems.addNumber("PD Temp", pdh::getTemperature);
        Constants.tab_subsystems.addNumber("PD Current", pdh::getTotalCurrent);
        Constants.tab_subsystems.addNumber("PD Joules", pdh::getTotalEnergy);

        autonChooser = new SendableChooser<>();
        autonChooser.setDefaultOption("Shoot and Drive", AutonCommand.State.SHOOT_DRIVE);
        autonChooser.addOption("Shoot", AutonCommand.State.SHOOT);
        autonChooser.addOption("Nothing", AutonCommand.State.NOTHING);
        autonChooser.addOption("Path (WIP)", AutonCommand.State.PATH);
        Constants.tab_subsystems.add("Auton State", autonChooser);
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
        new JoystickButton(buttonBoard, 5)
                .whileHeld(new PivotCommand(systems, false));
        
        // Pivot Down
        new JoystickButton(buttonBoard, 2)
                .whileHeld(new PivotCommand(systems, true));

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
                .whileHeld(new ShootCommand(systems, Shooter.Velocity.REJECT));

        // Shoot 
        new JoystickButton(buttonBoard, 6)
                .whileHeld(new ShootCommand(systems, Shooter.Velocity.NORMAL));
        
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
        
        // Angler towards 90
        new JoystickButton(operator, LogitechExtreme3D.Button.FIVE.ordinal() + 1)
                .whenPressed(new AnglerCommand(systems, true));

        // Angler towards 0
        new JoystickButton(operator, LogitechExtreme3D.Button.SIX.ordinal() + 1)
                .whenPressed(new AnglerCommand(systems, false));

        // Climber Extend (Manual)
        new JoystickButton(buttonBoard, 16)
                .whileHeld(new ClimberExtendCommand(systems, false));

        // Climber Extend Reverse (Manual)
        new JoystickButton(buttonBoard, 13)
                .whileHeld(new ClimberExtendCommand(systems, true));

        // Climber Hinge (Manual)
        new JoystickButton(buttonBoard, 14)
                .whileHeld(new ClimberHingeCommand(systems, false));

        // Climber Hinge Reverse (Manual)
        new JoystickButton(buttonBoard, 8)
                .whileHeld(new ClimberHingeCommand(systems, true));

        // Pivot Calibration
        new JoystickButton(operator, LogitechExtreme3D.Button.TEN.ordinal() + 1)
                .whenPressed(() -> systems.getPivot().calibrateMode(true), systems.getPivot())
                .whenReleased(() -> {
                    systems.getPivot().calibrateMode(false);
                    systems.getPivot().reset();
                }, systems.getPivot());

        // Climber Extend Calibration
        new JoystickButton(operator, LogitechExtreme3D.Button.SEVEN.ordinal() + 1)
                .whenPressed(() -> systems.getClimber().getExtend().calibrateMode(true), systems.getClimber().getExtend())
                .whenReleased(() -> {
                    systems.getClimber().getExtend().calibrateMode(false);
                    systems.getClimber().getExtend().reset();
                }, systems.getClimber().getExtend());

        // Climber Hinge Calibration
        new JoystickButton(operator, LogitechExtreme3D.Button.EIGHT.ordinal() + 1)
                .whenPressed(() -> systems.getClimber().getHinge().calibrateMode(true), systems.getClimber().getHinge())
                .whenReleased(() -> {
                    systems.getClimber().getHinge().calibrateMode(false);
                    systems.getClimber().getHinge().reset();
                }, systems.getClimber().getHinge());
        
        // Aim/Vision
        new JoystickButton(buttonBoard, 0) // TODO: button number
                .whenHeld(new AimCommand(systems));

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
        return new AutonCommand(systems, autonChooser.getSelected());
    }

    public Systems getSystems() {
        return systems;
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
