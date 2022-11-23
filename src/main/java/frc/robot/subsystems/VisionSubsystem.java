// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubsystem extends SubsystemBase {

  private PhotonCamera frontCamera;

  private ShuffleboardTab visionSubsystemTab;

  private Optional<Transform3d> bestCameraToTarget;

  /** Creates a new VisonSubsystem. */
  public VisionSubsystem() {
    frontCamera = new PhotonCamera("front camera");

    visionSubsystemTab = Shuffleboard.getTab("VisionSubsystem");

    visionSubsystemTab.addNumber(
        "x",
        () ->
            bestCameraToTarget
                .map(Transform3d::getTranslation)
                .map(Translation3d::getX)
                .orElse(0d));
    visionSubsystemTab.addNumber(
        "y",
        () ->
            bestCameraToTarget
                .map(Transform3d::getTranslation)
                .map(Translation3d::getY)
                .orElse(0d));
    visionSubsystemTab.addNumber(
        "z",
        () ->
            bestCameraToTarget
                .map(Transform3d::getTranslation)
                .map(Translation3d::getZ)
                .orElse(0d));
    visionSubsystemTab.addNumber(
        "angle",
        () ->
            bestCameraToTarget.map(Transform3d::getRotation).map(Rotation3d::getAngle).orElse(0d));
    visionSubsystemTab.addBoolean("has target", () -> bestCameraToTarget.isPresent());
  }

  @Override
  public void periodic() {
    var result = frontCamera.getLatestResult();
    var bestTarget = result.getBestTarget();

    bestCameraToTarget =
        Optional.ofNullable(bestTarget).map(PhotonTrackedTarget::getBestCameraToTarget);
  }

  public Optional<Transform3d> getBestCameraToTarget() {
    return bestCameraToTarget;
  }
}
