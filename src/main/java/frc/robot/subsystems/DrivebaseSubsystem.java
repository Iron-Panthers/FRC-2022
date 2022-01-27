// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.Drive.*;

import com.google.errorprone.annotations.concurrent.GuardedBy;
import com.ironpanthers.robot.lib.UpdateManager.Updatable;
import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivebaseSubsystem extends SubsystemBase implements Updatable {
  /** Object for locking gyroscope access */
  private final Object sensorLock = new Object();

  @GuardedBy("sensorLock")
  private final AHRS navx = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX connected over MXP

  // should only be accessed for initialization tasks
  // FIXME adjust
  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(
          // Front right
          new Translation2d(Dims.TRACKWIDTH_METERS / 2.0, -Dims.WHEELBASE_METERS / 2.0),
          // Front left
          new Translation2d(Dims.TRACKWIDTH_METERS / 2.0, Dims.WHEELBASE_METERS / 2.0),
          // Back left
          new Translation2d(-Dims.TRACKWIDTH_METERS / 2.0, Dims.WHEELBASE_METERS / 2.0),
          // Back right
          new Translation2d(-Dims.TRACKWIDTH_METERS / 2.0, -Dims.WHEELBASE_METERS / 2.0));

  /** Object for locking kinematics related values */
  private final Object kinematicsLock = new Object();

  @GuardedBy("kinematicsLock")
  private final SwerveDriveOdometry swerveOdometry =
      new SwerveDriveOdometry(kinematics, navx.getRotation2d());

  @GuardedBy("kinematicsLock")
  private Pose2d pose = new Pose2d();

  /** Object for locking ChassisSpeeds demand updates */
  private final Object stateLock = new Object();

  @GuardedBy("stateLock")
  private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(); // defaults to zeros

  /** The modes of the drivebase subsystem */
  public enum Modes {
    DRIVE,
    DEFENSE,
  }

  @GuardedBy("stateLock")
  /** the current mode */
  private Modes mode = Modes.DRIVE;

  /* Logging values */
  private final NetworkTableEntry odometryXEntry;
  private final NetworkTableEntry odometryYEntry;
  private final NetworkTableEntry odometryAngleEntry;

  /** Contains each swerve module. Order: FR, FL, BL, BR. Or in Quadrants: I, II, III, IV */
  private final SwerveModule[] swerveModules;

  /**
   * initialize a falcon with a shuffleboard tab, and mk4 default gear ratio
   *
   * @param tab the shuffleboard tab to use
   * @param title the shuffleboard title
   * @param pos the shuffleboard x position, which is <b>multiplied by 2</b>
   * @param drive the drive motor port const
   * @param steer the steer motor port const
   * @param encoder the encoder port const
   * @param offset the steer offset const, found experimentally
   * @return an sds swerve module object
   */
  private SwerveModule createModule(
      ShuffleboardTab tab,
      String title,
      int pos,
      int drive,
      int steer,
      int encoder,
      double offset) {

    return Mk4SwerveModuleHelper.createFalcon500(
        tab.getLayout(title, BuiltInLayouts.kList).withSize(2, 4).withPosition(pos * 2, 0),
        Mk4SwerveModuleHelper.GearRatio.L2,
        drive,
        steer,
        encoder,
        offset);
  }

  /** Creates a new DrivebaseSubsystem. */
  public DrivebaseSubsystem() {
    synchronized (sensorLock) {
      navx.getAngle(); // FIXME
    }

    ShuffleboardTab tab = Shuffleboard.getTab("Drivebase");

    final SwerveModule frontRightModule =
        createModule(
            tab,
            "Front Right Module #1",
            1,
            Modules.FrontRight.DRIVE_MOTOR,
            Modules.FrontRight.STEER_MOTOR,
            Modules.FrontRight.STEER_ENCODER,
            Modules.FrontRight.STEER_OFFSET);

    final SwerveModule frontLeftModule =
        createModule(
            tab,
            "Front Left Module #2",
            0,
            Modules.FrontLeft.DRIVE_MOTOR,
            Modules.FrontLeft.STEER_MOTOR,
            Modules.FrontLeft.STEER_ENCODER,
            Modules.FrontLeft.STEER_OFFSET);

    final SwerveModule backLeftModule =
        createModule(
            tab,
            "Back Left Module #3",
            2,
            Modules.BackLeft.DRIVE_MOTOR,
            Modules.BackLeft.STEER_MOTOR,
            Modules.BackLeft.STEER_ENCODER,
            Modules.BackLeft.STEER_OFFSET);

    final SwerveModule backRightModule =
        createModule(
            tab,
            "Back Right Module #4",
            3,
            Modules.BackRight.DRIVE_MOTOR,
            Modules.BackRight.STEER_MOTOR,
            Modules.BackRight.STEER_ENCODER,
            Modules.BackRight.STEER_OFFSET);

    swerveModules = // modules are always initialized and passed in this order
        new SwerveModule[] {frontRightModule, frontLeftModule, backLeftModule, backRightModule};

    odometryXEntry = tab.add("X", 0.0).withPosition(0, 0).withSize(1, 1).getEntry();
    odometryYEntry = tab.add("Y", 0.0).withPosition(0, 1).withSize(1, 1).getEntry();
    odometryAngleEntry = tab.add("Angle", 0.0).withPosition(0, 2).withSize(1, 1).getEntry();
    tab.addNumber(
        "Rotation Speed rad/s",
        () -> {
          ChassisSpeeds speeds;
          synchronized (stateLock) {
            speeds = this.chassisSpeeds;
          }
          if (speeds == null) return 0.0;

          return speeds.omegaRadiansPerSecond;
        });
  }

  /** Return the current pose estimation of the robot */
  public Pose2d getPose() {
    synchronized (kinematicsLock) {
      return pose;
    }
  }

  /**
   * Tells the subsystem to drive, and puts the state machine in drive mode
   *
   * @param chassisSpeeds the speed of the chassis desired
   */
  public void drive(ChassisSpeeds chassisSpeeds) {
    synchronized (stateLock) {
      this.mode = Modes.DRIVE;
      this.chassisSpeeds = chassisSpeeds;
    }
  }

  public void resetGyroAngle(Rotation2d angle) {
    synchronized (sensorLock) {
      // FIXME test
      navx.setAngleAdjustment(getGyroscopeRotation().rotateBy(angle.unaryMinus()).getDegrees());
    }
  }

  /** Sets the gyro angle to zero, resetting the forward direction */
  public void zeroGyroscope() {
    resetGyroAngle(Rotation2d.fromDegrees(0.0));
  }

  public Rotation2d getGyroscopeRotation() {
    synchronized (sensorLock) {
      if (navx.isMagnetometerCalibrated()) {
        // We will only get valid fused headings if the magnetometer is calibrated
        return Rotation2d.fromDegrees(navx.getFusedHeading());
      }

      // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes
      // the angle increase.
      return Rotation2d.fromDegrees(360.0 - navx.getYaw());
    }
  }

  private void updateOdometry(double time) {
    SwerveModuleState[] moduleStates = new SwerveModuleState[swerveModules.length];
    for (int i = 0; i < swerveModules.length; i++) {
      SwerveModule module = swerveModules[i];
      moduleStates[i] =
          new SwerveModuleState(
              module.getDriveVelocity(), Rotation2d.fromDegrees(module.getSteerAngle()));
    }

    Rotation2d angle;
    synchronized (sensorLock) {
      angle = getGyroscopeRotation();
    }

    synchronized (kinematicsLock) {
      this.pose = swerveOdometry.updateWithTime(time, angle, moduleStates);
    }
  }

  /**
   * gets the current mode of the drivebase subsystem state machine
   *
   * @return the current mode
   */
  public Modes getMode() {
    synchronized (stateLock) {
      return mode;
    }
  }

  /**
   * Angles the swerve modules in a cross shape, to make the robot hard to push. This function sets
   * the state machine to defense mode, so it only needs to be called once
   */
  public void setDefenseMode() {
    synchronized (stateLock) {
      mode = Modes.DEFENSE;
    }
  }

  public void updateModules(ChassisSpeeds chassisSpeeds, Modes mode) {
    if (chassisSpeeds == null) {
      chassisSpeeds = new ChassisSpeeds();
    }

    switch (mode) {
      case DRIVE:
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

        // sets swerve module speeds and angles, for each swerve module, using kinematics
        for (int i = 0; i < swerveModules.length; i++) {
          swerveModules[i].set(
              states[i].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
              states[i].angle.getRadians());
        }
        break;
      case DEFENSE:
        int angle = 45;
        for (SwerveModule module : swerveModules) {
          // the *= -1 operation multiplies the current variable by -1, stores it, and also returns
          // the
          // value. We can use this to alternate between 45 and -45 for each module.
          module.set(0, angle *= -1);
        }
        break;
      default:
        break;
    }
  }

  @Override
  public void update(double time, double dt) {
    updateOdometry(time);

    ChassisSpeeds speeds;
    Modes mode;
    // Optional<ChassisSpeeds> trajectorySpeeds = blahblah
    // if has trajectory signal
    // update drive signal accordingly
    // else
    synchronized (stateLock) {
      speeds = this.chassisSpeeds;
      mode = this.mode;
    }
    // end else
    updateModules(speeds, mode);
  }

  @Override
  public void periodic() {
    Pose2d pose = getPose();
    odometryXEntry.setDouble(pose.getX());
    odometryYEntry.setDouble(pose.getY());
    odometryAngleEntry.setDouble(pose.getRotation().getDegrees());
    // end
  }
}
