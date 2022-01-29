// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.Drive;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DefenseModeCommand;
import frc.robot.commands.RotateAngleDriveCommand;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.util.Util;
import java.util.function.DoubleSupplier;
import java.util.function.IntFunction;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final DrivebaseSubsystem drivebaseSubsystem = new DrivebaseSubsystem();

  private final XboxController nick = new XboxController(0);
  private final XboxController will = new XboxController(1);

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
            () -> (-modifyAxis(nick.getLeftY()) * Drive.MAX_VELOCITY_METERS_PER_SECOND),
            () -> (-modifyAxis(nick.getLeftX()) * Drive.MAX_VELOCITY_METERS_PER_SECOND),
            () -> (-modifyAxis(nick.getRightX()) * Drive.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND)));

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

    new Button(nick::getStartButton).whenPressed(drivebaseSubsystem::zeroGyroscope);
    new Button(nick::getLeftBumper).whenHeld(new DefenseModeCommand(drivebaseSubsystem));
    // these are flipped because the joystick is the opposite of intuition yay
    DoubleSupplier translationXSupplier =
        () -> (-modifyAxis(nick.getLeftY()) * Drive.MAX_VELOCITY_METERS_PER_SECOND);
    DoubleSupplier translationYSupplier =
        () -> (-modifyAxis(nick.getLeftX()) * Drive.MAX_VELOCITY_METERS_PER_SECOND);

    IntFunction<RotateAngleDriveCommand> rotCommand =
        angle ->
            RotateAngleDriveCommand.fromRobotRelative(
                drivebaseSubsystem, translationXSupplier, translationYSupplier, angle);

    new Button(nick::getYButton).whenPressed(rotCommand.apply(0));
    new Button(nick::getBButton).whenPressed(rotCommand.apply(90));
    new Button(nick::getAButton).whenPressed(rotCommand.apply(180));
    new Button(nick::getXButton).whenPressed(rotCommand.apply(270));
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
    value = Util.deadband(value, 0.2);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}
