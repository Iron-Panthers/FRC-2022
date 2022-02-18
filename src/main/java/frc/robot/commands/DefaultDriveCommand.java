// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.util.Util;
import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends CommandBase {
  private final DrivebaseSubsystem drivebaseSubsystem;

  private final DoubleSupplier translationXSupplier;
  private final DoubleSupplier translationYSupplier;
  private final DoubleSupplier rotationXSupplier;
  private final DoubleSupplier rotationYSupplier;

  private double angle = 0;
  private static final double[] angles = {0, 45, 90, 135, 180, 225, 270, 315};

  /** Creates a new DefaultDriveCommand. */
  public DefaultDriveCommand(
      DrivebaseSubsystem drivebaseSubsystem,
      DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier,
      DoubleSupplier rotationXSupplier,
      DoubleSupplier rotationYSupplier) {

    this.drivebaseSubsystem = drivebaseSubsystem;
    this.translationXSupplier = translationXSupplier;
    this.translationYSupplier = translationYSupplier;
    this.rotationXSupplier = rotationXSupplier;
    this.rotationYSupplier = rotationYSupplier;

    addRequirements(drivebaseSubsystem);
  }

  @Override
  public void initialize() {
    angle = drivebaseSubsystem.getGyroscopeRotation().getDegrees();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double x = translationXSupplier.getAsDouble();
    double y = translationYSupplier.getAsDouble();
    double rotX = rotationXSupplier.getAsDouble();
    double rotY = rotationYSupplier.getAsDouble();

    // if stick magnitude is greater then .5
    if (Math.sqrt(Math.pow(rotX, 2) + Math.pow(rotY, 2)) > .5) {
      angle = Util.angleSnap(Util.vectorToAngle(-rotX, -rotY), angles);
    }

    drivebaseSubsystem.driveAngle(new Pair<>(x, y), angle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivebaseSubsystem.drive(new ChassisSpeeds());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
