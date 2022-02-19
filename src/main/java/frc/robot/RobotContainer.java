// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.Drive;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DefenseModeCommand;
import frc.robot.commands.HaltDriveCommandsCommand;
import frc.robot.commands.RotateVelocityDriveCommand;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.util.ControllerUtil;
import frc.util.MacUtil;
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

  private final XboxController nick = new XboxController(1);
  private final XboxController will = new XboxController(0);

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
            () -> (-modifyAxis(will.getLeftX()) * Drive.MAX_VELOCITY_METERS_PER_SECOND),
            will::getRightY,
            will::getRightX));

    SmartDashboard.putBoolean("is comp bot", MacUtil.IS_COMP_BOT);

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

    new Button(will::getStartButton)
        .whenPressed(new InstantCommand(drivebaseSubsystem::zeroGyroscope, drivebaseSubsystem));
    new Button(will::getLeftBumper).whenHeld(new DefenseModeCommand(drivebaseSubsystem));

    new Button(will::getLeftStickButton)
        .whenPressed(new HaltDriveCommandsCommand(drivebaseSubsystem));

    DoubleSupplier rotation =
        () ->
            ControllerUtil.deadband((-will.getRightTriggerAxis() + will.getLeftTriggerAxis()), .1);
    DoubleSupplier rotationVelocity =
        () -> rotation.getAsDouble() * Drive.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;

    new Button(() -> Math.abs(rotation.getAsDouble()) > 0)
        .whenHeld(
            new RotateVelocityDriveCommand(
                drivebaseSubsystem,
                () -> (-modifyAxis(will.getLeftY()) * Drive.MAX_VELOCITY_METERS_PER_SECOND),
                () -> (-modifyAxis(will.getLeftX()) * Drive.MAX_VELOCITY_METERS_PER_SECOND),
                rotationVelocity));

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

    // intake balls
    new Button(nick::getAButton).whenHeld(intakeCommand.apply(IntakeSubsystem.Modes.INTAKE));
    // eject unwanted balls
    new Button(nick::getBButton).whenHeld(intakeCommand.apply(IntakeSubsystem.Modes.EJECT));
    // shoot balls
    new Button(nick::getYButton).whenHeld(intakeCommand.apply(IntakeSubsystem.Modes.OUTTAKE));
    // stop everything
    new Button(nick::getXButton).whenHeld(intakeCommand.apply(IntakeSubsystem.Modes.OFF));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
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
