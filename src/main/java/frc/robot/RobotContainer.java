// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.Drive;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.Constants.Arm;
import frc.robot.autonomous.commands.BaselineAutoSequence;
import frc.robot.autonomous.commands.OffsideTwoCargoAutoSequence;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DefenseModeCommand;
import frc.robot.commands.HaltDriveCommandsCommand;
import frc.robot.commands.RotateVectorDriveCommand;
import frc.robot.commands.RotateVelocityDriveCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.util.ControllerUtil;
import frc.util.MacUtil;
import frc.util.Util;
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
  private final ArmSubsystem armSubsystem = new ArmSubsystem();

  /** controller 1 */
  private final XboxController nick = new XboxController(1);
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
    //           armSubsystem.setPercentOutput(ControllerUtil.deadband(nick.getLeftY(), .2));
    //         },
    //         (interupted) -> {},
    //         () -> false,
    //         armSubsystem));

    SmartDashboard.putBoolean("is comp bot", MacUtil.IS_COMP_BOT);

    // Configure the button bindings
    configureButtonBindings();

    // Create and put autonomous selector to dashboard
    setupAutonomousCommands();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

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

    // will controller intakes (temporary)
    new Button(will::getRightBumper).whenHeld(intakeCommand.apply(IntakeSubsystem.Modes.INTAKE));
    new Button(will::getLeftBumper).whenHeld(intakeCommand.apply(IntakeSubsystem.Modes.OUTTAKE));

    DoubleFunction<InstantCommand> armAngleCommand =
        angle -> new InstantCommand(() -> armSubsystem.setAngle(angle), armSubsystem);

    // Arm to high goal
    new Button(nick::getLeftBumper)
        .whenPressed(armAngleCommand.apply(Arm.Setpoints.OUTTAKE_HIGH_POSITION));

    // Arm to intake position
    new Button(nick::getRightBumper)
        .whenPressed(armAngleCommand.apply(Arm.Setpoints.INTAKE_POSITION));

    // hold arm up for sideways intake
    new Button(nick::getStartButton)
        .whenHeld(
            new FunctionalCommand(
                () -> armSubsystem.setAngle(Arm.Setpoints.INTAKE_HIGHER_POSITION),
                () -> {},
                (interrupted) -> armSubsystem.setAngle(Arm.Setpoints.INTAKE_POSITION),
                () -> false,
                armSubsystem));

    // intake balls
    new Button(nick::getAButton).whenHeld(intakeCommand.apply(IntakeSubsystem.Modes.INTAKE));
    // shoot balls
    new Button(nick::getYButton).whenHeld(intakeCommand.apply(IntakeSubsystem.Modes.OUTTAKE));
    // fast outtake
    new Button(nick::getXButton).whenHeld(intakeCommand.apply(IntakeSubsystem.Modes.OUTTAKE_FAST));

    // eject left side
    new Button(
            () ->
                nick.getBButton()
                    && nick.getLeftTriggerAxis() > .5
                    && nick.getRightTriggerAxis() <= .5)
        .whenHeld(intakeCommand.apply(IntakeSubsystem.Modes.EJECT_LEFT));
    // eject right side
    new Button(
            () ->
                nick.getBButton()
                    && nick.getLeftTriggerAxis() <= .5
                    && nick.getRightTriggerAxis() > .5)
        .whenHeld(intakeCommand.apply(IntakeSubsystem.Modes.EJECT_RIGHT));
    // eject everything
    new Button(
            () ->
                nick.getBButton()
                    && nick.getLeftTriggerAxis() > .5
                    && nick.getRightTriggerAxis() > .5)
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

    Shuffleboard.getTab("DriverView").add(autoSelector);
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
