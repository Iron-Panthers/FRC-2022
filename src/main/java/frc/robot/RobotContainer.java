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
import frc.robot.commands.EjectLeftManualCommand;
import frc.robot.commands.EjectRightManualCommand;
import frc.robot.commands.IntakeManualCommand;
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
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  


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

    new Button(remy::getAButton)
        .whileHeld(new IntakeManualCommand(intakeSubsystem, Constants.Intake.intakePower));

    new Button(remy::getYButton)
        .whileHeld(new IntakeManualCommand(intakeSubsystem, Constants.Intake.outtakePower));
    
    new Button(() -> remy.getLeftTriggerAxis() > 0.5 )
        .whileHeld(new EjectLeftManualCommand(intakeSubsystem, Constants.Intake.intakePower));

    new Button(() -> remy.getRightTriggerAxis() > 0.5 )
        .whileHeld(new EjectRightManualCommand(intakeSubsystem, Constants.Intake.intakePower));

    
  }

  /**
   * Adds all autonomous routines to the autoSelector, and places the autoSelector on Shuffleboard.
   */
  private void setupAutonomousCommands() { 
  }

}
