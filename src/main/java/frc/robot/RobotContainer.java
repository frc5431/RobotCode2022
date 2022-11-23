// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import org.photonvision.PhotonCamera;

import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.UsbCameraInfo;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.*;
import frc.robot.commands.subsystems.*;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Shooter;
import frc.robot.util.CameraCalc;
import frc.team5431.titan.core.joysticks.LogitechExtreme3D;
import frc.team5431.titan.core.joysticks.utils.CompassPOV;
import frc.team5431.titan.core.misc.Calc;

import static edu.wpi.first.wpilibj2.command.Commands.*;

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

    private final CommandXboxController driver = new CommandXboxController(0);
  //private final Joystick vjoy = new vjoy(4);
    private final CommandXboxController operator = new CommandXboxController(1);
    private final LogitechExtreme3D manualJoystick = new LogitechExtreme3D(2);

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
                        systems,
                        () -> modifyAxis(-driver.getLeftY()) * Drivebase.MAX_VELOCITY_METERS_PER_SECOND,
                        () -> modifyAxis(-driver.getLeftX()) * Drivebase.MAX_VELOCITY_METERS_PER_SECOND,
                        () -> modifyAxis(-driver.getRightX()) * Drivebase.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
        ));

        setDefaultAnglerCommand();
        // unsetDefaultAnglerCommand();

        systems.getClimber().getExtend().setDefaultCommand(systems.getClimber().getExtend().runClimberCommand(() -> {
            return modifyAxis(operator.getRightTriggerAxis())
                  - modifyAxis(operator.getLeftTriggerAxis());
        }));

        // Configure the button bindings
        configureButtonBindings();

        camera.setLED(Constants.DEFAULT_LED_MODE);
        camera.setPipelineIndex(Constants.VISION_PIPELINE_INDEX);
        PhotonCamera.setVersionCheckEnabled(false);

        Constants.tab_subsystems.addString("LED Pattern", () -> systems.getLed().getPattern().toString())
                .withPosition(4, 2)
                .withSize(2, 1);

        Constants.tab_subsystems.addBoolean("DIO result", () -> systems.getUpperFeederSensor().get())
                .withPosition(0, 6)
                .withSize(2, 1);

        Constants.tab_subsystems.addNumber("Distance (m)", () -> CameraCalc.getDistanceMeters(camera))
                .withPosition(6, 0)
                .withSize(2, 1);

        autonChooser = new SendableChooser<>();
        // autonChooser.addOption("One Ball (no taxi)", AutonCommand.State.ONE_BALL);
        autonChooser.setDefaultOption("Two Ball (time)", AutonCommand.State.TWO_BALL);
        autonChooser.addOption("Three Ball (path)", AutonCommand.State.THREE_BALL);
        autonChooser.addOption("Four Ball (time)", AutonCommand.State.FOUR_BALL);
        autonChooser.addOption("Five Ball (path)", AutonCommand.State.FIVE_BALL);
        autonChooser.addOption("NO BALL", AutonCommand.State.NO_BALL);
        autonChooser.addOption("Test Path", AutonCommand.State.TEST_PATH);
        autonChooser.addOption("Just Path", AutonCommand.State.JUST_PATH);
        Constants.tab_subsystems.add("Auton State", autonChooser)
                .withPosition(9, 6)
                .withSize(3, 1);

        try {
            UsbCameraInfo[] cameras = UsbCamera.enumerateUsbCameras();
            for (int i = 0; i < cameras.length; i++) {
                // Constants.tab_subsystems.add(new UsbCamera(cameras[i].name, cameras[i].path))
                //         .withSize(6, 6)
                //         .withPosition(2+i*11, 4);
            }
        } catch (Exception e) {}

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
        driver.y().onTrue(runOnce(drivebase::zeroGyroscope));
        
        // D-Pad cardinal directions
        driver.povUp().whileTrue(run(
                () -> drivebase.driveController(new ChassisSpeeds(Drivebase.MAX_VELOCITY_METERS_PER_SECOND, 0, 0)), drivebase));
        driver.povDown().whileTrue(run(
                () -> drivebase.driveController(new ChassisSpeeds(-Drivebase.MAX_VELOCITY_METERS_PER_SECOND, 0, 0)), drivebase));
        driver.povLeft().whileTrue(run(
                    () -> drivebase.driveController(new ChassisSpeeds(0, -Drivebase.MAX_VELOCITY_METERS_PER_SECOND, 0)), drivebase));
        driver.povRight().whileTrue(run(
                    () -> drivebase.driveController(new ChassisSpeeds(0, Drivebase.MAX_VELOCITY_METERS_PER_SECOND, 0)), drivebase));
        
        // Browse LED patterns
        driver.back().onTrue(new LEDCommand(systems, LEDCommand.COMMAND.PREV));
        driver.start().onTrue(new LEDCommand(systems, LEDCommand.COMMAND.NEXT));
        
        // Lock to Hub Mode:tm:
        driver.a().onTrue(runOnce( () -> { Drivebase.lockedToHub = !Drivebase.lockedToHub; } ));
        
        // Intake (Manual)
        operator.povRight().whileTrue(systems.getIntake().runIntakeCommand(false));
        
        // Intake Reverse (Manual)
        operator.x().whileTrue(
                systems.getIntake().runIntakeCommand(true)
                    .alongWith(systems.getFeeder().feedEverythingCommand(true)));
        
        // Pivot Up
        operator.povUp().whileTrue(systems.getPivot().runPivotCommand(false));
        
        // Pivot Down
        operator.povDown().whileTrue(systems.getPivot().runPivotCommand(true));

        // Trigger/slider Shoot
        manualJoystick.getButton(LogitechExtreme3D.Button.TRIGGER)
                .whileTrue(systems.getShooter().runShooterCommand(
                        () -> Calc.map(
                                manualJoystick.getRawAxis(LogitechExtreme3D.Axis.SLIDER), 
                                        1.0, -1.0, 
                                        0, Shooter.MAX_VELOCITY)));
        
        // Floor Intake
        operator.a().toggleOnTrue(new FloorIntakeCommand(systems));
        
        // Reject
        // operator.back().whileTrue(
        //         new ShootCommand(systems, Shooter.VELOCITY_REJECT)
        //             .alongWith(new AnglerCommand(systems, AnglerCommand.COMMAND.SET, 0.287)));

        // Shoot & Aim
        operator.b().whileTrue(ShootCommands.timedFeedShootWithAimCommand(systems, () -> CameraCalc.calculateRPM(camera), true, true));
        
        // Shoot only
        operator.y().whileTrue(ShootCommands.shootCalcRPMCommand(systems));
        
        // Feed Both Up
        manualJoystick.getButton(CompassPOV.NORTH)
                .whileTrue(systems.getFeeder().feedEverythingCommand(false));
        
        // Feed Both Down
        manualJoystick.getButton(CompassPOV.SOUTH)
                .whileTrue(systems.getFeeder().feedEverythingCommand(true));
        
        // Angler towards 90 (lower angler)
        manualJoystick.getButton(LogitechExtreme3D.Button.FIVE)
                .onTrue(new AnglerCommand(systems, true));

        // Angler towards 0 (raise angler)
        manualJoystick.getButton(LogitechExtreme3D.Button.SIX)
                .onTrue(new AnglerCommand(systems, false));

        manualJoystick.getButton(LogitechExtreme3D.Button.NINE)
                .toggleOnTrue(startEnd(
                    this::unsetDefaultAnglerCommand,
                    this::setDefaultAnglerCommand));

        // Shoot from Hub (manual)
        manualJoystick.getButton(LogitechExtreme3D.Button.THREE)
                .whileTrue(ShootCommands.angleAndShootCommand(systems, ShootCommands.ShootPresets.HUB));

        // Shoot from Safe Zone (manual)
        manualJoystick.getButton(LogitechExtreme3D.Button.FOUR)
                .whileTrue(ShootCommands.angleAndShootCommand(systems, ShootCommands.ShootPresets.SAFEZONE));

        // Auto Climb
        operator.back().onTrue(new AutoClimbCommand(systems, operator.back()::getAsBoolean));

        // Climber Extend (Manual) (moved to default)
        // new JoystickButton(buttonBoard, 16)
        //         .whileHeld(systems.getClimber().getExtend().runClimberCommand(false));

        // Climber Extend Reverse (Manual) (moved to default)
        // new JoystickButton(buttonBoard, 13)
        //         .whileHeld(systems.getClimber().getExtend().runClimberCommand(true));

        // Climber Hinge In (Manual)
        operator.leftBumper().whileTrue(systems.getClimber().getHinge().runClimberCommand(true));

        // Climber Hinge Out (Manual)
        operator.rightBumper().whileTrue(systems.getClimber().getHinge().runClimberCommand(false));

        // Pivot Calibration
        manualJoystick.getButton(LogitechExtreme3D.Button.TEN)
                .onTrue(runOnce(() -> systems.getPivot().calibrateMode(true), systems.getPivot()))
                .onFalse(runOnce(() -> {
                    systems.getPivot().calibrateMode(false);
                    systems.getPivot().reset();
                }, systems.getPivot()));

        // Climber Extend Calibration
        manualJoystick.getButton(LogitechExtreme3D.Button.SEVEN)
                .onTrue(runOnce(() -> systems.getClimber().getExtend().calibrateMode(true), systems.getClimber().getExtend()))
                .onFalse(runOnce(() -> {
                    systems.getClimber().getExtend().calibrateMode(false);
                    systems.getClimber().getExtend().reset();
                }, systems.getClimber().getExtend()));

        // Climber Hinge Calibration
        manualJoystick.getButton(LogitechExtreme3D.Button.EIGHT)
                .onTrue(runOnce(() -> systems.getClimber().getHinge().calibrateMode(true), systems.getClimber().getHinge()))
                .onFalse(runOnce(() -> {
                    systems.getClimber().getHinge().calibrateMode(false);
                    systems.getClimber().getHinge().reset();
                }, systems.getClimber().getHinge()));
        
        // Calibrate All
        operator.start()
                .onTrue(runOnce(() -> {
                    systems.getPivot().calibrateMode(true);
                    systems.getClimber().getExtend().calibrateMode(true);
                    systems.getClimber().getHinge().calibrateMode(true);
                }, systems.getPivot(), systems.getClimber().getExtend(), systems.getClimber().getHinge()))
                .onFalse(runOnce(() -> {
                    systems.getPivot().calibrateMode(false);
                    systems.getPivot().reset();
                    systems.getClimber().getExtend().calibrateMode(false);
                    systems.getClimber().getExtend().reset();
                    systems.getClimber().getHinge().calibrateMode(false);
                    systems.getClimber().getHinge().reset();
                }, systems.getPivot(), systems.getClimber().getExtend(), systems.getClimber().getHinge()));
        
        // Aim/Vision
        // new JoystickButton(buttonBoard, 11)
        // buttonController.getButton(Xbox.Button.Y)
        //         .whenHeld(new AimCommand(systems));

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
        // systems.getLed().set(0.87); // blue
        // systems.getLed().stop();
        // camera.setLED(VisionLEDMode.kOff);
    }

    public void enabledInit() {
        drivebase.setNeutralModeDrive(NeutralMode.Brake);
        drivebase.setNeutralModeSteer(NeutralMode.Brake);
        camera.setLED(Constants.DEFAULT_LED_MODE);
        systems.getLed().set(Constants.LEDPATTERN_DEFAULT);
        
        System.out.println("VideoSource size: " + VideoSource.enumerateSources().length);
        for (VideoSource vs : VideoSource.enumerateSources()) {
            System.out.println(String.format("Camera ID: %s - Name: %s - Description: %s", vs.getHandle(), vs.getName(), vs.getDescription()));
        }
    }

    public void unsetDefaultAnglerCommand() {
        Command def = systems.getAngler().getDefaultCommand();
        if (def == null) return;

        CommandScheduler.getInstance().unregisterSubsystem(systems.getAngler());
        CommandScheduler.getInstance().registerSubsystem(systems.getAngler());

        def.cancel();
    }

    public void setDefaultAnglerCommand() {
        systems.getAngler().setDefaultCommand(
                new AnglerCommand(systems, AnglerCommand.COMMAND.SET, () -> CameraCalc.calculateAngler(camera) ) {
                        @Override
                        public boolean isFinished() { return false; }
                }
        );
    }

    public void teleopPeriodic() {
        // System.out.println("DIO Sensor: " + systems.getUpperFeederSensor().get());
    }
}
