// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.*;
import frc.robot.commands.subsystems.*;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Shooter;
import frc.robot.util.CameraCalc;
import frc.team5431.titan.core.joysticks.LogitechExtreme3D;
import frc.team5431.titan.core.joysticks.Xbox;
import frc.team5431.titan.core.misc.Calc;
import frc.team5431.titan.core.robot.JoystickButton;

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

//     private final PowerDistribution pdh = new PowerDistribution();

    private final Xbox driver = new Xbox(0);
  //private final Joystick vjoy = new vjoy(4);
    private final Xbox buttonController = new Xbox(1);
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

        systems.getAngler().setDefaultCommand(new AnglerCommand(systems, AnglerCommand.COMMAND.SET, () -> CameraCalc.calculateAngler(camera) ) {
                @Override
                public boolean isFinished() { return false; }
        });

        systems.getClimber().getExtend().setDefaultCommand(new ClimberExtendCommand(systems, () -> {
            return modifyAxis(buttonController.getRawAxis(Xbox.Axis.TRIGGER_RIGHT))
                  - modifyAxis(buttonController.getRawAxis(Xbox.Axis.TRIGGER_LEFT));
        }));

        // Configure the button bindings
        configureButtonBindings();

        camera.setLED(Constants.DEFAULT_LED_MODE);
        camera.setPipelineIndex(Constants.VISION_PIPELINE_INDEX);

        Constants.tab_subsystems.addBoolean("DIO result", () -> systems.getUpperFeederSensor().get());

        Constants.tab_subsystems.addNumber("Distance (m)", () -> CameraCalc.getDistanceMeters(camera));

        autonChooser = new SendableChooser<>();
        // autonChooser.addOption("One Ball (no taxi)", AutonCommand.State.ONE_BALL);
        autonChooser.setDefaultOption("Two Ball (taxi)", AutonCommand.State.TWO_BALL);
        autonChooser.addOption("Three Ball", AutonCommand.State.THREE_BALL);
        autonChooser.addOption("Test Path", AutonCommand.State.TEST_PATH);
        autonChooser.addOption("Just Path", AutonCommand.State.JUST_PATH);
        Constants.tab_subsystems.add("Auton State", autonChooser);


        Constants.tab_commands.add(CommandScheduler.getInstance());
        Constants.tab_commands.add("Path 0 Reset Odom", AutonCommand.commandResetAuton(systems, AutonCommand.PATHS[0]));
        Constants.tab_commands.add("Path 0 - to first ball", new PathCommand(systems, AutonCommand.PATHS[0]));
        Constants.tab_commands.add("Path 1 - to third ball", new PathCommand(systems, AutonCommand.PATHS[1]));
        Constants.tab_commands.add("Path 2 - to terminal", new PathCommand(systems, AutonCommand.PATHS[2]));
        Constants.tab_commands.add("Path 3 - to shoot", new PathCommand(systems, AutonCommand.PATHS[3]));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // Y button zeros the gyroscope
        new JoystickButton(driver, Xbox.Button.Y)
                // No requirements because we don't need to interrupt anything
                .whenPressed(drivebase::zeroGyroscope);
        
        // D-Pad cardinal directions
        new POVButton(driver, 0) // up
                .whileHeld(
                    () -> drivebase.driveController(new ChassisSpeeds(Drivebase.MAX_VELOCITY_METERS_PER_SECOND, 0, 0)), drivebase);
        new POVButton(driver, 180) // down
                .whileHeld(
                    () -> drivebase.driveController(new ChassisSpeeds(-Drivebase.MAX_VELOCITY_METERS_PER_SECOND, 0, 0)), drivebase);
        new POVButton(driver, 270) // right
                .whileHeld(
                    () -> drivebase.driveController(new ChassisSpeeds(0, Drivebase.MAX_VELOCITY_METERS_PER_SECOND, 0)), drivebase);
        new POVButton(driver, 90) // left
                .whileHeld(
                    () -> drivebase.driveController(new ChassisSpeeds(0, -Drivebase.MAX_VELOCITY_METERS_PER_SECOND, 0)), drivebase);
        
        // Intake (Manual)
        // new JoystickButton(buttonBoard, 7)
        new POVButton(buttonController, 270)
                .whileHeld(new IntakeCommand(systems, false));
        
        // Intake Reverse (Manual)
        // new JoystickButton(buttonBoard, 3)
        new JoystickButton(buttonController, Xbox.Button.X)
                .whileHeld(
                        new IntakeCommand(systems, true)
                            .alongWith(new FeedEverything(systems, true))
                );
        
        // Pivot Up
        // new JoystickButton(buttonBoard, 5)
        new POVButton(buttonController, 0)
                .whileHeld(new PivotCommand(systems, false));
        
        // Pivot Down
        // new JoystickButton(buttonBoard, 2)
        new POVButton(buttonController, 180)
                .whileHeld(new PivotCommand(systems, true));

        // Trigger/slider Shoot
        new JoystickButton(operator, LogitechExtreme3D.Button.TRIGGER)
                .whileHeld(new ShooterCommand(systems, 
                        () -> Calc.map(
                                operator.getRawAxis(LogitechExtreme3D.Axis.SLIDER), 
                                        1.0, -1.0, 
                                        0, Shooter.MAX_VELOCITY)));
        
        // Floor Intake
        // new JoystickButton(buttonBoard, 4)
        new JoystickButton(buttonController, Xbox.Button.A)
                .toggleWhenPressed(new FloorIntakeCommand(systems));
        
        // Reject
        // new JoystickButton(buttonBoard, 1)
        new JoystickButton(buttonController, Xbox.Button.BACK)
                .whileHeld(new ShootCommand(systems, Shooter.Velocity.REJECT));

        // Shoot 
        // new JoystickButton(buttonBoard, 6)
        new JoystickButton(buttonController, Xbox.Button.B)
                .whenHeld(new ShootPlusCommand(systems));
        
        // Feed Both Up
        new POVButton(operator, 0)
                .whileHeld(new FeedEverything(systems, false));
        
        // Feed Both Down
        new POVButton(operator, 180)
                .whileHeld(new FeedEverything(systems, true));
        
        // Angler towards 90 (lower angler)
        new JoystickButton(operator, LogitechExtreme3D.Button.FIVE)
                .whenPressed(new AnglerCommand(systems, true));

        // Angler towards 0 (raise angler)
        new JoystickButton(operator, LogitechExtreme3D.Button.SIX)
                .whenPressed(new AnglerCommand(systems, false));

        // Shoot from Hub (manual)
        new JoystickButton(operator, LogitechExtreme3D.Button.THREE)
                .whenHeld(new AngleAndShootCommand(systems, AngleAndShootCommand.Position.HUB));

        // Shoot from Safe Zone (manual)
        new JoystickButton(operator, LogitechExtreme3D.Button.FOUR)
                .whenHeld(new AngleAndShootCommand(systems, AngleAndShootCommand.Position.SAFEZONE));

        // Climber Extend (Manual) (moved to default)
        // new JoystickButton(buttonBoard, 16)
        //         .whileHeld(new ClimberExtendCommand(systems, false));

        // Climber Extend Reverse (Manual) (moved to default)
        // new JoystickButton(buttonBoard, 13)
        //         .whileHeld(new ClimberExtendCommand(systems, true));

        // Climber Hinge (Manual)
        // new JoystickButton(buttonBoard, 14)
        new JoystickButton(buttonController, Xbox.Button.BUMPER_L)
                .whileHeld(new ClimberHingeCommand(systems, false));

        // Climber Hinge Reverse (Manual)
        // new JoystickButton(buttonBoard, 8)
        new JoystickButton(buttonController, Xbox.Button.BUMPER_R)
                .whileHeld(new ClimberHingeCommand(systems, true));

        // Pivot Calibration
        new JoystickButton(operator, LogitechExtreme3D.Button.TEN)
                .whenPressed(() -> systems.getPivot().calibrateMode(true), systems.getPivot())
                .whenReleased(() -> {
                    systems.getPivot().calibrateMode(false);
                    systems.getPivot().reset();
                }, systems.getPivot());

        // Climber Extend Calibration
        new JoystickButton(operator, LogitechExtreme3D.Button.SEVEN)
                .whenPressed(() -> systems.getClimber().getExtend().calibrateMode(true), systems.getClimber().getExtend())
                .whenReleased(() -> {
                    systems.getClimber().getExtend().calibrateMode(false);
                    systems.getClimber().getExtend().reset();
                }, systems.getClimber().getExtend());

        // Climber Hinge Calibration
        new JoystickButton(operator, LogitechExtreme3D.Button.EIGHT)
                .whenPressed(() -> systems.getClimber().getHinge().calibrateMode(true), systems.getClimber().getHinge())
                .whenReleased(() -> {
                    systems.getClimber().getHinge().calibrateMode(false);
                    systems.getClimber().getHinge().reset();
                }, systems.getClimber().getHinge());
        
        // Calibrate All
        new JoystickButton(buttonController, Xbox.Button.START)
                .whenPressed(() -> {
                    systems.getPivot().calibrateMode(true);
                    systems.getClimber().getExtend().calibrateMode(true);
                    systems.getClimber().getHinge().calibrateMode(true);
                }, systems.getPivot(), systems.getClimber().getExtend(), systems.getClimber().getHinge())
                .whenReleased(() -> {
                    systems.getPivot().calibrateMode(false);
                    systems.getPivot().reset();
                    systems.getClimber().getExtend().calibrateMode(false);
                    systems.getClimber().getExtend().reset();
                    systems.getClimber().getHinge().calibrateMode(false);
                    systems.getClimber().getHinge().reset();
                }, systems.getPivot(), systems.getClimber().getExtend(), systems.getClimber().getHinge());
        
        // Aim/Vision
        // new JoystickButton(buttonBoard, 11)
        new JoystickButton(buttonController, Xbox.Button.Y)
                .whenHeld(new AimCommand(systems));

        // Stop All
        // new JoystickButton(buttonBoard, 12)
        //         .whileHeld(new StopAllCommand(systems));

        // new JoystickButton(operator, LogitechExtreme3D.Button.TWELVE)
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

        value = value * value * value;

        // Square the axis
        // value = Math.copySign(value * value, value);

        return value;
    }

    public void disabledInit()  {
        drivebase.stop();
        drivebase.setNeutralModeDrive(NeutralMode.Coast);
        drivebase.setNeutralModeSteer(NeutralMode.Coast);
        // camera.setLED(VisionLEDMode.kOff);
    }

    public void enabledInit() {
        drivebase.setNeutralModeDrive(NeutralMode.Brake);
        drivebase.setNeutralModeSteer(NeutralMode.Brake);
        camera.setLED(Constants.DEFAULT_LED_MODE);
    }

    public void teleopPeriodic() {
        // System.out.println("DIO Sensor: " + systems.getUpperFeederSensor().get());
    }
}
