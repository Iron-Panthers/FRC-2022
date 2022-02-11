// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.Drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DefenseModeCommand;
import frc.robot.commands.FollowTrajectoryCommand;
import frc.robot.commands.HaltDriveCommandsCommand;
import frc.robot.commands.RotateAngleDriveCommand;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.util.ControllerUtil;
import frc.util.Layer;
import frc.util.MacUtil;
import java.util.List;
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

    new Button(nick::getStartButton).whenPressed(drivebaseSubsystem::zeroGyroscope);
    new Button(nick::getLeftBumper).whenHeld(new DefenseModeCommand(drivebaseSubsystem));
    // these are flipped because the joystick is the opposite of intuition yay
    DoubleSupplier translationXSupplier =
        () -> (-modifyAxis(nick.getLeftY()) * Drive.MAX_VELOCITY_METERS_PER_SECOND);
    DoubleSupplier translationYSupplier =
        () -> (-modifyAxis(nick.getLeftX()) * Drive.MAX_VELOCITY_METERS_PER_SECOND);

    IntFunction<RotateAngleDriveCommand> rotCommand =
        angle ->
            new RotateAngleDriveCommand(
                drivebaseSubsystem, translationXSupplier, translationYSupplier, angle);

    IntFunction<RotateAngleDriveCommand> relRotCommand =
        angle ->
            RotateAngleDriveCommand.fromRobotRelative(
                drivebaseSubsystem, translationXSupplier, translationYSupplier, angle);

    Layer rightBumper = new Layer(nick::getRightBumper);

    // when the bumper is not held, field relative rotation
    rightBumper.off(nick::getYButton).whenPressed(rotCommand.apply(0));
    rightBumper.off(nick::getBButton).whenPressed(rotCommand.apply(270));
    rightBumper.off(nick::getAButton).whenPressed(rotCommand.apply(180));
    rightBumper.off(nick::getXButton).whenPressed(rotCommand.apply(90));

    // otherwise, rotate robot
    rightBumper.on(nick::getYButton).whenPressed(relRotCommand.apply(180)); // flip left
    rightBumper.on(nick::getBButton).whenPressed(relRotCommand.apply(-90));
    rightBumper.on(nick::getAButton).whenPressed(relRotCommand.apply(-180)); // flip right
    rightBumper.on(nick::getXButton).whenPressed(relRotCommand.apply(90));

    new Button(nick::getLeftStickButton)
        .whenPressed(new HaltDriveCommandsCommand(drivebaseSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    TrajectoryConfig config =
        new TrajectoryConfig(1, 0.5)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(drivebaseSubsystem.getKinematics());

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, -1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            config);

    return new FollowTrajectoryCommand(exampleTrajectory, drivebaseSubsystem);
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
