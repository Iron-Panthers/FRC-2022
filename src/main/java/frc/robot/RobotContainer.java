// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.Drive;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.Constants.Arm;
import frc.robot.autonomous.commands.AutoTestSequence;
import frc.robot.autonomous.commands.GreedyOnsideAutoSequence;
import frc.robot.autonomous.commands.OffsideTwoCargoAutoSequence;
import frc.robot.autonomous.commands.OnsideFourSequence;
import frc.robot.autonomous.commands.OnsideOneBallSteal;
import frc.robot.autonomous.commands.OnsideThreeSequence;
// import frc.robot.autonomous.commands.TaxiAutoSequence;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DefenseModeCommand;
import frc.robot.commands.ElevatorAutomatedCommand;
import frc.robot.commands.ElevatorManualCommand;
import frc.robot.commands.ElevatorPositionCommand;
import frc.robot.commands.ForceIntakeModeCommand;
import frc.robot.commands.HaltDriveCommandsCommand;
import frc.robot.commands.InstantSetIntakeModeCommand;
import frc.robot.commands.PreciseArmCommand;
import frc.robot.commands.RotateVectorDriveCommand;
import frc.robot.commands.RotateVelocityDriveCommand;
import frc.robot.commands.VibrateControllerCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.util.ControllerUtil;
import frc.util.Layer;
import frc.util.MacUtil;
import frc.util.Util;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleFunction;
import java.util.function.DoubleSupplier;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final DrivebaseSubsystem drivebaseSubsystem = new DrivebaseSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final ArmSubsystem armSubsystem = new ArmSubsystem(elevatorSubsystem);

  /** controller 1 */
  private final XboxController jason = new XboxController(1);
  /** controller 1 climb layer */
  private final Layer jasonLayer = new Layer(jason::getRightBumper);
  /** controller 0 */
  private final XboxController will = new XboxController(0);

  /** the sendable chooser to select which auto to run. */
  private final SendableChooser<Command> autoSelector = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    drivebaseSubsystem.setDefaultCommand(
        new DefaultDriveCommand(
            drivebaseSubsystem,
            () -> (-modifyAxis(will.getLeftY()) * Drive.MAX_VELOCITY_METERS_PER_SECOND),
            () -> (-modifyAxis(will.getLeftX()) * Drive.MAX_VELOCITY_METERS_PER_SECOND)));

    // armSubsystem.setDefaultCommand(
    //     new FunctionalCommand(
    //         () -> {},
    //         () -> {
    //           armSubsystem.setPercentOutput(ControllerUtil.deadband(jason.getLeftY(), .2));
    //         },
    //         (interupted) -> {},
    //         () -> false,
    //         armSubsystem));

    SmartDashboard.putBoolean("is comp bot", MacUtil.IS_COMP_BOT);

    // Configure the button bindings
    configureButtonBindings();

    // Create and put autonomous selector to dashboard
    setupAutonomousCommands();

    // start the camera server and configure the cameras
    setupCameras();
  }

  public void containerTeleopInit() {
    // runs when teleop happens
    CommandScheduler.getInstance().schedule(new VibrateControllerCommand(jason, 5, .5));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // vibrate jason controller when in layer
    jasonLayer.whenChanged(
        (enabled) -> {
          final double power = enabled ? .1 : 0;
          jason.setRumble(RumbleType.kLeftRumble, power);
          jason.setRumble(RumbleType.kRightRumble, power);
        });

    new Button(will::getStartButton)
        .whenPressed(new InstantCommand(drivebaseSubsystem::zeroGyroscope, drivebaseSubsystem));
    new Button(will::getLeftBumper).whenHeld(new DefenseModeCommand(drivebaseSubsystem));

    new Button(will::getLeftStickButton)
        .whenPressed(new HaltDriveCommandsCommand(drivebaseSubsystem));

    DoubleSupplier rotation =
        () ->
            ControllerUtil.deadband((will.getRightTriggerAxis() + -will.getLeftTriggerAxis()), .1);
    DoubleSupplier rotationVelocity =
        () ->
            rotation.getAsDouble()
                * Drive.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
                * .5 /* half speed trigger rotation per will */;

    new Button(() -> Math.abs(rotation.getAsDouble()) > 0)
        .whenHeld(
            new RotateVelocityDriveCommand(
                drivebaseSubsystem,
                /* drive joystick "y" is passed to x because controller is inverted */
                () -> (-modifyAxis(will.getLeftY()) * Drive.MAX_VELOCITY_METERS_PER_SECOND),
                () -> (-modifyAxis(will.getLeftX()) * Drive.MAX_VELOCITY_METERS_PER_SECOND),
                rotationVelocity));

    new Button(
            () ->
                Util.vectorMagnitude(will.getRightY(), will.getRightX())
                    > Drive.ROTATE_VECTOR_MAGNITUDE)
        .whenPressed(
            new RotateVectorDriveCommand(
                drivebaseSubsystem,
                () -> (-modifyAxis(will.getLeftY()) * Drive.MAX_VELOCITY_METERS_PER_SECOND),
                () -> (-modifyAxis(will.getLeftX()) * Drive.MAX_VELOCITY_METERS_PER_SECOND),
                will::getRightY,
                will::getRightX));

    // Elevator preset position buttons
    jasonLayer
        .on(jason::getBButton)
        .whenHeld(
            new ElevatorPositionCommand(
                elevatorSubsystem, Constants.Elevator.MAX_HEIGHT)); // Elevator goes to top
    jasonLayer
        .on(jason::getXButton)
        .whenHeld(
            new ElevatorPositionCommand(
                elevatorSubsystem, Constants.Elevator.MIN_HEIGHT)); // Elevator goes to bottom
    jasonLayer
        .on(jason::getAButton)
        .whenHeld(new ElevatorAutomatedCommand(elevatorSubsystem)); // Elevator does auto sequence

    // Elevator Manual controls
    // jasonLayer
    //     .on(jason::getYButton)
    //     .whenHeld(
    //         new ElevatorManualCommand(
    //             elevatorSubsystem, Constants.Elevator.RATE)); // Makes elevator go up manually
    // jasonLayer
    //     .on(jason::getAButton)
    //     .whenHeld(
    //         new ElevatorManualCommand(
    //             elevatorSubsystem, -Constants.Elevator.RATE)); // Makes elevator go down manually

    jasonLayer
        .on(() -> Math.abs(jason.getLeftY()) >= .4)
        .whenHeld(
            new ElevatorManualCommand(
                elevatorSubsystem, modifyAxis(ControllerUtil.deadband(jason.getLeftY(), .4))));

    // new FunctionalCommand(
    //     () -> {},
    //     () ->
    //         elevatorSubsystem.setPercent(
    //             modifyAxis(ControllerUtil.deadband(jason.getLeftY(), .4))),
    //     (interrupted) -> {
    //       elevatorSubsystem.setPercent(0);
    //     },
    //     () -> false,
    //     elevatorSubsystem));

    DoubleFunction<InstantCommand> armAngleCommand =
        angle -> new InstantCommand(() -> armSubsystem.setAngle(angle), armSubsystem);

    BooleanSupplier armToHeightButton =
        jasonLayer.off(() -> Util.vectorMagnitude(jason.getLeftY(), jason.getLeftX()) > .8);

    // Arm to high goal
    new Button(() -> armToHeightButton.getAsBoolean() && jason.getLeftY() <= 0)
        .whenPressed(armAngleCommand.apply(Arm.Setpoints.OUTTAKE_HIGH_POSITION));

    // Arm to intake position
    new Button(() -> armToHeightButton.getAsBoolean() && jason.getLeftY() > 0)
        .whenPressed(armAngleCommand.apply(Arm.Setpoints.INTAKE_POSITION));

    // when in climb mode, precise angle adjustment stick
    new Button(jasonLayer.getLayerSwitch())
        .whenPressed(
            new PreciseArmCommand(
                armSubsystem,
                () ->
                    /** negative so it maps to arm motion */
                    -ControllerUtil.deadband(jason.getRightY(), .2)));

    // hold arm up for sideways intake
    new Button(jason::getLeftStickButton)
        .whenHeld(
            new FunctionalCommand(
                () -> armSubsystem.setAngle(Arm.Setpoints.INTAKE_HIGHER_POSITION),
                () -> {},
                (interrupted) -> armSubsystem.setAngle(Arm.Setpoints.INTAKE_POSITION),
                () -> false,
                armSubsystem));

    // intake balls
    jasonLayer
        .off(jason::getAButton)
        .whenHeld(new ForceIntakeModeCommand(intakeSubsystem, IntakeSubsystem.Modes.INTAKE));
    // score into low from fender
    jasonLayer
        .off(jason::getXButton)
        .whenHeld(
            new ForceIntakeModeCommand(intakeSubsystem, IntakeSubsystem.Modes.INTAKE_FORCEFUL));
    // score into high from fender
    jasonLayer
        .off(jason::getLeftBumper)
        .whenPressed(
            new InstantSetIntakeModeCommand(
                intakeSubsystem, IntakeSubsystem.Modes.OUTTAKE_HIGH_ALL));

    // stop everything
    jasonLayer
        .off(jason::getBButton)
        .whenPressed(new InstantSetIntakeModeCommand(intakeSubsystem, IntakeSubsystem.Modes.OFF));

    // eject left side
    jasonLayer
        .off(() -> jason.getLeftTriggerAxis() > .5 && jason.getRightTriggerAxis() <= .5)
        .whenHeld(new ForceIntakeModeCommand(intakeSubsystem, IntakeSubsystem.Modes.EJECT_LEFT));
    // eject right side
    jasonLayer
        .off(() -> jason.getLeftTriggerAxis() <= .5 && jason.getRightTriggerAxis() > .5)
        .whenHeld(new ForceIntakeModeCommand(intakeSubsystem, IntakeSubsystem.Modes.EJECT_RIGHT));
    // eject everything
    jasonLayer
        .off(() -> jason.getLeftTriggerAxis() > .5 && jason.getRightTriggerAxis() > .5)
        .whenHeld(new ForceIntakeModeCommand(intakeSubsystem, IntakeSubsystem.Modes.EJECT_ALL));
  }

  /**
   * Adds all autonomous routines to the autoSelector, and places the autoSelector on Shuffleboard.
   */
  private void setupAutonomousCommands() {
    Shuffleboard.getTab("DriverView")
        .addString("NOTES", () -> "Onside is right side. We got this Danny!");
    autoSelector.setDefaultOption(
        "[OLD] OffsideAuto2",
        new OffsideTwoCargoAutoSequence(
            3, // Optimal values per 2022-03-08 test (ih)
            1.5,
            drivebaseSubsystem.getKinematics(),
            armSubsystem,
            drivebaseSubsystem,
            intakeSubsystem));

    autoSelector.setDefaultOption(
        "[OLD] OnsideOneBallSteal (choose this maddie, we got this)",
        new OnsideOneBallSteal(
            3, // Optimal values per 2022-03-08 test (ih)
            1.5,
            drivebaseSubsystem.getKinematics(),
            armSubsystem,
            drivebaseSubsystem,
            intakeSubsystem));

    autoSelector.addOption(
        "[OLD] OnsideAuto3",
        new OnsideThreeSequence(
            3,
            1.5,
            drivebaseSubsystem.getKinematics(),
            armSubsystem,
            drivebaseSubsystem,
            intakeSubsystem));

    autoSelector.addOption(
        "[OLD] OnsideAuto4",
        new OnsideFourSequence(
            4, // m/s
            2.75, // m/s2
            drivebaseSubsystem.getKinematics(),
            armSubsystem,
            drivebaseSubsystem,
            intakeSubsystem));

    autoSelector.addOption(
        "[NEW] AutoTest",
        new AutoTestSequence(
            2, // m/s
            1, // m/s2
            drivebaseSubsystem.getKinematics(),
            armSubsystem,
            drivebaseSubsystem,
            intakeSubsystem));
    /*
    autoSelector.addOption(
        "[NEW] Taxi and Disrupt",
        new TaxiAutoSequence(
            4, // m/s
            1, // m/s2
            drivebaseSubsystem.getKinematics(),
            armSubsystem,
            drivebaseSubsystem,
            intakeSubsystem));
    */

    autoSelector.addOption(
        "[OLD] DONOTUSE",
        new GreedyOnsideAutoSequence(
            4, // m/s
            5, // m/s2
            drivebaseSubsystem.getKinematics(),
            armSubsystem,
            drivebaseSubsystem,
            intakeSubsystem));

    autoSelector.addOption(
        "[NEW] OnsideThreeBallSequence",
        new OnsideThreeSequence(
            3, // m/s
            1, // m/s2
            drivebaseSubsystem.getKinematics(),
            armSubsystem,
            drivebaseSubsystem,
            intakeSubsystem));

    Shuffleboard.getTab("DriverView")
        .add("auto selector", autoSelector)
        .withSize(4, 1)
        .withPosition(6, 0);
  }

  private void setupCameras() {
    UsbCamera intakeCamera = CameraServer.startAutomaticCapture("intake camera", 0);
    intakeCamera.setVideoMode(PixelFormat.kMJPEG, 160, 120, 15);
    Shuffleboard.getTab("DriverView")
        .add(intakeCamera)
        .withSize(6, 6)
        .withPosition(0, 0)
        .withProperties(Map.of("show controls", false));

    // UsbCamera fieldCamera = CameraServer.startAutomaticCapture("field camera", 1);
    // fieldCamera.setVideoMode(PixelFormat.kMJPEG, 160, 120, 7);
    // Shuffleboard.getTab("DriverView")
    //     .add(fieldCamera)
    //     .withSize(6, 6)
    //     .withPosition(6, 0)
    //     .withProperties(Map.of("show controls", false));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoSelector.getSelected();
  }

  /**
   * applies deadband and squares axis
   *
   * @param value the axis value to be modified
   * @return the modified axis values
   */
  private static double modifyAxis(double value) {
    // Deadband
    value = ControllerUtil.deadband(value, 0.07);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}
