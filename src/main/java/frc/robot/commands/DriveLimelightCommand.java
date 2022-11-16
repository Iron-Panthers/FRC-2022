// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.trajectory.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.List;


import frc.robot.subsystems.DrivebaseSubsystem;



public class DriveLimelightCommand extends CommandBase {
  private final DrivebaseSubsystem drivebaseSubsystem;


  private NetworkTable table;
  private NetworkTableEntry tx;
  private NetworkTableEntry ta;
  private NetworkTableEntry ty; 


  private PIDController xController;
  private PIDController yController;

  private ShuffleboardTab shuffleboard;
  /**
   * Creates a new DriveTimed.
   *
   * @param timeS The number of inches the robot will drive
   * @param speedY The speed at which the robot will drive in Y (front+/back-)
   * @param speedX The speed at which the robot will drive in X (left+/right-)
   * @param speedRot The speed at which the robot will rotate
   * @param drive The drive subsystem on which this command will run
   */
  public DriveLimelightCommand(
      DrivebaseSubsystem drivebaseSubsystem, NetworkTable table) {

    this.drivebaseSubsystem = drivebaseSubsystem;
    this.table = table; 

    shuffleboard = Shuffleboard.getTab("limelight-rightup");
    
    xController = new PIDController(0.1, 0, 0);
    yController = new PIDController(0.1, 0, 0);
   // yController.setSetpoint(0);

    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");

    shuffleboard.addNumber("X", () -> tx.getDouble(0.0));
    shuffleboard.addNumber("Y", () -> ty.getDouble(0.0));
    shuffleboard.addNumber("A", () -> ta.getDouble(0.0));
  }

  @Override
  public void initialize() {
    //m_drive.resetEncoders();

  }

  @Override
  public void execute() {


    double ldifferenceX = -xController.calculate(tx.getDouble(0.0));
    ldifferenceX = MathUtil.clamp(ldifferenceX, -1, 1);

    double ldifferenceY = -yController.calculate(ta.getDouble(0.0));
    ldifferenceY = MathUtil.clamp(ldifferenceY, -1, 1);

    drivebaseSubsystem.drive(new ChassisSpeeds(ldifferenceY, 0, ldifferenceX));
  }


  @Override
  public void end(boolean interrupted) {
   
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}