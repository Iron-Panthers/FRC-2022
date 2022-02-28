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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.Constants.Arm;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DefenseModeCommand;
import frc.robot.commands.FollowTrajectoryCommand;
import frc.robot.commands.HaltDriveCommandsCommand;
import frc.robot.commands.RotateVectorDriveCommand;
import frc.robot.commands.RotateVelocityDriveCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.util.ControllerUtil;
import frc.util.MacUtil;
import frc.util.Util;
import java.util.List;
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

    // Arm to highest - high goal
    new Button(nick::getAButton)
        .whenPressed(
            new InstantCommand(
                () -> armSubsystem.setAngle(Arm.Setpoints.OUTTAKE_HIGH_POSITION), armSubsystem));
    // Arm to low goal - shortcut for engineering
    new Button(nick::getXButton)
        .whenPressed(
            new InstantCommand(
                () -> armSubsystem.setAngle(Arm.Setpoints.OUTTAKE_LOW_POSITION),
                armSubsystem)); // FIX ME -> What angle should this be?
    // Arm to lowest
    new Button(nick::getBButton)
        .whenPressed(
            new InstantCommand(
                () -> armSubsystem.setAngle(Arm.Setpoints.INTAKE_POSITION), armSubsystem));

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
