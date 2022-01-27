// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import com.google.errorprone.annotations.concurrent.GuardedBy;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import java.util.Optional;

/** Follower for trajectory of type T */
public abstract class TrajectoryFollower<T> {
  private final Object trajectoryLock = new Object();

  /** The trajectory that is currently being followed. null if no trajectory is being followed. */
  @GuardedBy("trajectoryLock")
  private Trajectory currentTrajectory = null;

  /**
   * The time that the current trajectory started to be followed. NaN if the trajectory has not been
   * started yet.
   */
  @GuardedBy("trajectoryLock")
  private double startTime = Double.NaN;

  /**
   * Calculates the output required to follow the trajectory.
   *
   * @param currentPose the current pose of the robot
   * @param trajectory the trajectory to follow
   * @param time the amount of time that has elapsed since the current trajectory started to be
   *     followed
   * @param dt the amount of time that has elapsed since the update loop was last ran
   * @return the chassis speeds required to follow the trajectory
   */
  protected abstract T calculateDriveSignal(
      Pose2d currentPose, Trajectory trajectory, double time, double dt);

  /**
   * Gets if the follower is done following the path.
   *
   * @return true if the path is done
   */
  protected abstract boolean isFinished();

  protected abstract void reset();

  public final void cancel() {
    synchronized (trajectoryLock) {
      currentTrajectory = null;
    }
  }

  public final void follow(Trajectory trajectory) {
    synchronized (trajectoryLock) {
      currentTrajectory = trajectory;
      startTime = Double.NaN;
    }
  }

  /** Gets the current trajectory that is being followed, if applicable. */
  public final Optional<Trajectory> getCurrentTrajectory() {
    synchronized (trajectoryLock) {
      return Optional.ofNullable(currentTrajectory);
    }
  }

  /**
   * Gets the desired output for the robot at this time.
   *
   * @param currentPose the current pose of the robot
   * @param time the current time
   * @param dt the time since update was last called
   * @return the output required to follow the current path if applicable
   */
  public final Optional<T> update(Pose2d currentPose, double time, double dt) {
    Trajectory trajectory;
    double timeSinceStart;
    synchronized (trajectoryLock) {
      // Return empty if no trajectory is being followed
      if (currentTrajectory == null) {
        return Optional.empty();
      }
      // If the trajectory has not been started, update the start time and reset the follower state
      if (Double.isNaN(startTime)) {
        startTime = time;
        reset();
      } else if (isFinished()) {
        currentTrajectory = null;
        return Optional.empty();
      }
      trajectory = currentTrajectory;
      timeSinceStart = time - startTime;
    }
    T speeds = calculateDriveSignal(currentPose, trajectory, timeSinceStart, dt);
    return Optional.of(speeds);
  }
}
