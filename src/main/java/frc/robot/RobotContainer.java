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
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.Constants.Arm;
import frc.robot.autonomous.commands.BaselineAutoSequence;
import frc.robot.autonomous.commands.OffsideTwoCargoAutoSequence;
import frc.robot.autonomous.commands.OnsideThreeCargoAutoSequence;
import frc.robot.autonomous.commands.OnsideTwoCargoAutoSequence;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DefenseModeCommand;
import frc.robot.commands.ElevatorPositionCommand;
import frc.robot.commands.HaltDriveCommandsCommand;
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
import java.util.function.BooleanSupplier;
import java.util.function.DoubleFunction;
import java.util.function.DoubleSupplier;
import java.util.function.Function;

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
  private final ArmSubsystem armSubsystem =
      new ArmSubsystem(
          elevatorSubsystem /* the arm subsystem reacts to the state of the elevator subsystem */);

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

    /**
     * this curried start end command calls setMode with the passed mode, then calls next mode when
     * the command is stopped
     */
    Function<IntakeSubsystem.Modes, StartEndCommand> intakeCommand =
        mode ->
            new StartEndCommand(
                () -> intakeSubsystem.setMode(mode), intakeSubsystem::nextMode, intakeSubsystem);

    // Elevator preset position buttons
    jasonLayer
        .on(jason::getBButton)
        .whenPressed(
            new ElevatorPositionCommand(
                elevatorSubsystem, Constants.Elevator.maxHeight)); // Elevator goes to top
    jasonLayer
        .on(jason::getXButton)
        .whenPressed(
            new ElevatorPositionCommand(
                elevatorSubsystem, Constants.Elevator.minHeight)); // Elevator goes to bottom

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
            new FunctionalCommand(
                () -> {},
                () ->
                    elevatorSubsystem.setPercent(
                        modifyAxis(ControllerUtil.deadband(jason.getLeftY(), .4))),
                (interrupted) -> {
                  elevatorSubsystem.setPercent(0);
                },
                () -> false,
                elevatorSubsystem));

    // will controller intakes (temporary)
    new Button(will::getRightBumper).whenHeld(intakeCommand.apply(IntakeSubsystem.Modes.INTAKE));
    new Button(will::getLeftBumper).whenHeld(intakeCommand.apply(IntakeSubsystem.Modes.OUTTAKE));

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

    BooleanSupplier armToHeightButtonForClimb =
        jasonLayer.on(() -> Util.vectorMagnitude(jason.getRightY(), jason.getRightX()) > .8);

    // Arm to high goal
    new Button(() -> armToHeightButtonForClimb.getAsBoolean() && jason.getRightY() <= 0)
        .whenPressed(armAngleCommand.apply(Arm.Setpoints.CLIMB_POSITION));

    // Arm to intake position
    new Button(() -> armToHeightButtonForClimb.getAsBoolean() && jason.getRightY() > 0)
        .whenPressed(armAngleCommand.apply(Arm.Setpoints.INTAKE_POSITION));

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
    jasonLayer.off(jason::getAButton).whenHeld(intakeCommand.apply(IntakeSubsystem.Modes.INTAKE));
    // fender shot
    jasonLayer.off(jason::getYButton).whenHeld(intakeCommand.apply(IntakeSubsystem.Modes.OUTTAKE));
    // far shot
    jasonLayer
        .off(jason::getXButton)
        .whenHeld(intakeCommand.apply(IntakeSubsystem.Modes.OUTTAKE_FAST));
    // stop everything
    jasonLayer.off(jason::getBButton).whenPressed(intakeCommand.apply(IntakeSubsystem.Modes.OFF));

    // eject left side
    jasonLayer
        .off(() -> jason.getLeftTriggerAxis() > .5 && jason.getRightTriggerAxis() <= .5)
        .whenHeld(intakeCommand.apply(IntakeSubsystem.Modes.EJECT_LEFT));
    // eject right side
    jasonLayer
        .off(() -> jason.getLeftTriggerAxis() <= .5 && jason.getRightTriggerAxis() > .5)
        .whenHeld(intakeCommand.apply(IntakeSubsystem.Modes.EJECT_RIGHT));
    // eject everything
    jasonLayer
        .off(() -> jason.getLeftTriggerAxis() > .5 && jason.getRightTriggerAxis() > .5)
        .whenHeld(intakeCommand.apply(IntakeSubsystem.Modes.EJECT_ALL));
  }

  /**
   * Adds all autonomous routines to the autoSelector, and places the autoSelector on Shuffleboard.
   */
  private void setupAutonomousCommands() {
    autoSelector.setDefaultOption(
        "baseline auto",
        new BaselineAutoSequence(4, 2, drivebaseSubsystem.getKinematics(), drivebaseSubsystem));

    autoSelector.addOption(
        "offside two cargo",
        new OffsideTwoCargoAutoSequence(
            3, // Optimal values per 2022-03-08 test (ih)
            1.5,
            drivebaseSubsystem.getKinematics(),
            armSubsystem,
            drivebaseSubsystem,
            intakeSubsystem));

    autoSelector.addOption(
        "onside two cargo",
        new OnsideTwoCargoAutoSequence(
            3,
            1.5,
            drivebaseSubsystem.getKinematics(),
            armSubsystem,
            drivebaseSubsystem,
            intakeSubsystem));

    autoSelector.addOption(
        "onside three cargo",
        new OnsideThreeCargoAutoSequence(
            3,
            1.5,
            drivebaseSubsystem.getKinematics(),
            armSubsystem,
            drivebaseSubsystem,
            intakeSubsystem));

    Shuffleboard.getTab("DriverView").add("auto selector", autoSelector);
  }

  private void setupCameras() {
    UsbCamera intakeCamera = CameraServer.startAutomaticCapture("intake camera", 0);
    var work = intakeCamera.setVideoMode(PixelFormat.kMJPEG, 176, 144, 15);
    SmartDashboard.putBoolean("camera set workded", work);
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
