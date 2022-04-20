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
import frc.robot.commands.ArmManualCommand;
import frc.robot.subsystems.ArmSubsystem; 
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
 
  private final ArmSubsystem armSubsystem = new ArmSubsystem();


  /** controller 1 */
  private final XboxController remy = new XboxController(1);
  /** controller 1 climb layer */
  //private final Layer jasonLayer = new Layer(jason::getRightBumper); 
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    // armSubsystem.setDefaultCommand(
    //     new FunctionalCommand(
    //         () -> {},
    //         () -> {
    //           armSubsystem.setPercentOutput(ControllerUtil.deadband(jason.getLeftY(), .2));
    //         },
    //         (interupted) -> {},
    //         () -> false,
    //         armSubsystem));

    configureButtonBindings();
 
  }

  public void containerTeleopInit() { 
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
   
    new Button(remy::getBButton)
        .whenHeld(new ArmManualCommand(armSubsystem, Constants.Arm.power));

    new Button(remy::getXButton)
    .whenHeld(new ArmManualCommand(armSubsystem, -Constants.Arm.power));
  }

  /**
   * Adds all autonomous routines to the autoSelector, and places the autoSelector on Shuffleboard.
   */
  private void setupAutonomousCommands() { 
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
