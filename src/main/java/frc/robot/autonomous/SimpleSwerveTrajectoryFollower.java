// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;

/** Implements a simple swerve trajectory follower. TODO test */
public class SimpleSwerveTrajectoryFollower extends TrajectoryFollower<ChassisSpeeds> {
  private PIDController xController;
  private PIDController yController;
  private ProfiledPIDController angleController;

  private Trajectory.State lastState = null;
  private boolean finished = false;
  /* Variable to track if calculateDriveSignal has run once yet */
  protected boolean firstRun = false;

  public SimpleSwerveTrajectoryFollower(
      PIDController xController, PIDController yController, ProfiledPIDController angleController) {
    this.xController = xController;
    this.yController = yController;
    this.angleController = angleController;
    angleController.enableContinuousInput(0, 2 * Math.PI); // FIXME: try -pi..pi
  }

  @Override
  protected ChassisSpeeds calculateDriveSignal(
      Pose2d currentPose, Trajectory trajectory, double time, double dt) {
    if (time > trajectory.getTotalTimeSeconds()) {
      finished = true;
      return new ChassisSpeeds();
    }

    // there is still time left!
    lastState = trajectory.sample(time);
    if (firstRun) {
      angleController.reset(currentPose.getRotation().getRadians());
      firstRun = false;
    }
    Pose2d poseRef = lastState.poseMeters;
    double linearVelocityRefMeters = lastState.velocityMetersPerSecond;
    double xFF = linearVelocityRefMeters * poseRef.getRotation().getCos();
    double yFF = linearVelocityRefMeters * poseRef.getRotation().getSin();
    double angleFF =
        angleController.calculate(
            currentPose.getRotation().getRadians(), 0 /*radians FIXME no magic number */);

    // FOR TESTING: DISABLE STRAFE PID COMPENSATION BY UN-COMMENTING THIS
    // return ChassisSpeeds.fromFieldRelativeSpeeds(xFF, yFF, angleFF, currentPose.getRotation());

    double xControllerEffort = xController.calculate(currentPose.getX(), poseRef.getX());
    double yControllerEffort = yController.calculate(currentPose.getY(), poseRef.getY());

    return ChassisSpeeds.fromFieldRelativeSpeeds(
        xFF + xControllerEffort, yFF + yControllerEffort, angleFF, currentPose.getRotation());
  }

  public Trajectory.State getLastState() {
    return lastState;
  }

  @Override
  protected boolean isFinished() {
    return finished;
  }

  @Override
  protected void reset() {
    xController.reset();
    yController.reset();
    angleController.reset(0);
    finished = false;
    firstRun = false;
  }
}
