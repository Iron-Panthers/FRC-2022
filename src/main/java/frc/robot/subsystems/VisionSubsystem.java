// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;

public class VisionSubsystem extends SubsystemBase {

  private PhotonCamera frontCamera;

  private ShuffleboardTab visionSubsystem;

  private Transform3d bestCameraToTarget;

  /** Creates a new VisonSubsystem. */
  public VisionSubsystem() {
    frontCamera = new PhotonCamera("front camera");

    visionSubsystem = Shuffleboard.getTab("VisionSubsystem");

    // create a group for the x, y, and z translation using bestCameraToTarget

    var listGroup =
        visionSubsystem.getLayout("bestCameraToTarget", BuiltInLayouts.kList).withSize(2, 4);

    listGroup.addNumber("x", () -> bestCameraToTarget.getTranslation().getX());
    listGroup.addNumber("y", () -> bestCameraToTarget.getTranslation().getY());
    listGroup.addNumber("z", () -> bestCameraToTarget.getTranslation().getZ());
    listGroup.addNumber("angle", () -> bestCameraToTarget.getRotation().getAngle());
  }

  @Override
  public void periodic() {
    var result = frontCamera.getLatestResult();
    var bestTarget = result.getBestTarget();

    bestCameraToTarget = bestTarget.getBestCameraToTarget();
  }

  public Transform3d getBestCameraToTarget() {
    return bestCameraToTarget;
  }
}
