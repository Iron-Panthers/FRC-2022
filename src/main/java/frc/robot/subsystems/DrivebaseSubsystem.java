// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.Drive.*;

import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.autonomous.SimpleSwerveTrajectoryFollower;
import java.util.Optional;

public class DrivebaseSubsystem extends SubsystemBase {
  private final SimpleSwerveTrajectoryFollower follower =
      new SimpleSwerveTrajectoryFollower(
          new PIDController(0.4, 0.0, 0.025),
          new PIDController(0.4, 0.0, 0.025),
          new ProfiledPIDController(
              1,
              0,
              0,
              new TrapezoidProfile.Constraints(
                  MAX_VELOCITY_METERS_PER_SECOND,
                  0.5 * MAX_VELOCITY_METERS_PER_SECOND))); // FIXME: replace with empirical or
  // smarter theoretical values

  public SimpleSwerveTrajectoryFollower getFollower() {
    return follower;
  }

  private final AHRS navx = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX connected over MXP

  /**
   * The kinematics object allows us to encode our relationship between desired speeds (represented
   * by a ChassisSpeeds object) and our actual drive outputs (what speeds and angles we apply to
   * each module)
   */
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

  /**
   * The SwerveDriveOdometry class allows us to estimate our vehicle position over time using the
   * power of :sparkles: math :sparkles:.
   */
  private final SwerveDriveOdometry swerveOdometry =
      new SwerveDriveOdometry(kinematics, navx.getRotation2d());

  /**
   * Keeps track of the current estimated pose (x,y,theta) of the robot, as estimated by odometry.
   */
  private Pose2d robotPose = new Pose2d();

  /** The current ChassisSpeeds goal for the drivetrain */
  private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(); // defaults to zeros

  /** The mode of the drivebase subsystem */
  public enum Modes {
    DRIVE, // For autonomous operation, the DRIVE mode is used -- no need to create another
    DEFENSE,
  }

  /** The current mode */
  private Modes mode = Modes.DRIVE;

  /* Logging values */
  private final NetworkTableEntry odometryXEntry;
  private final NetworkTableEntry odometryYEntry;
  private final NetworkTableEntry odometryAngleEntry;

  /** Contains each swerve module. Order: FR, FL, BL, BR. Or in Quadrants: I, II, III, IV */
  private final SwerveModule[] swerveModules;

  /**
   * The Shuffleboard tab which all things related to the drivebase can be put for easy access and
   * organization
   */
  private final ShuffleboardTab tab = Shuffleboard.getTab("Drivebase");

  /**
   * Initialize a falcon with a shuffleboard tab, and mk4 default gear ratio
   *
   * @param tab the shuffleboard tab to use
   * @param title the shuffleboard title
   * @param pos the shuffleboard x position, which is <b>multiplied by 2</b>
   * @param drive the drive motor port const
   * @param steer the steer motor port const
   * @param encoder the encoder port const
   * @param offset the steer offset const, found experimentally
   * @return SDS/swerve-lib SwerveModule object
   */
  private SwerveModule createModule(
      String title, int pos, int drive, int steer, int encoder, double offset) {
    // NOTE: our team uses the MK4 configuration with L2 gearing and Falcon 500s
    // if this changes, update the helper/method/GearRatio used, as needed.
    return Mk4SwerveModuleHelper.createFalcon500(
        tab.getLayout(title, BuiltInLayouts.kList).withSize(2, 4).withPosition(pos * 2, 0),
        Mk4SwerveModuleHelper.GearRatio.L2,
        drive,
        steer,
        encoder,
        offset);
  }

  /**
   * Helper to add an entry easily to the drivebase subsystem Shuffleboard.
   *
   * <p>If you want to use similar logic in another class, please rewrite -- this will always write
   * values to the "Drivebase" ShuffleboardTab, which is not desirable behavior for anything that
   * isn't physically or logically a part of the drivebase.
   *
   * <p>Another note -- you may wish to attach an anonymous "lambda" function instead (which avoids
   * the need to use a NetworkTableEntry handle). I'll add this documentation later because I am
   * lazy.
   *
   * @param label The label for the NetworkTableEntry.
   * @param row The row to place the value in (needed so that widgets do not overlap)
   * @return The NetworkTableEntry handle which can be updated to affect the underlying value.
   */
  private NetworkTableEntry createEntry(String label, int row) {
    return tab.add(label, 0.0).withPosition(0, row).withSize(1, 1).getEntry();
  }

  /** Creates a new DrivebaseSubsystem. */
  public DrivebaseSubsystem() {
    final SwerveModule frontRightModule =
        createModule(
            "Front Right Module #1",
            1,
            Modules.FrontRight.DRIVE_MOTOR,
            Modules.FrontRight.STEER_MOTOR,
            Modules.FrontRight.STEER_ENCODER,
            Modules.FrontRight.STEER_OFFSET);

    final SwerveModule frontLeftModule =
        createModule(
            "Front Left Module #2",
            0,
            Modules.FrontLeft.DRIVE_MOTOR,
            Modules.FrontLeft.STEER_MOTOR,
            Modules.FrontLeft.STEER_ENCODER,
            Modules.FrontLeft.STEER_OFFSET);

    final SwerveModule backLeftModule =
        createModule(
            "Back Left Module #3",
            2,
            Modules.BackLeft.DRIVE_MOTOR,
            Modules.BackLeft.STEER_MOTOR,
            Modules.BackLeft.STEER_ENCODER,
            Modules.BackLeft.STEER_OFFSET);

    final SwerveModule backRightModule =
        createModule(
            "Back Right Module #4",
            3,
            Modules.BackRight.DRIVE_MOTOR,
            Modules.BackRight.STEER_MOTOR,
            Modules.BackRight.STEER_ENCODER,
            Modules.BackRight.STEER_OFFSET);

    swerveModules = // modules are always initialized and passed in this order
        new SwerveModule[] {frontRightModule, frontLeftModule, backLeftModule, backRightModule};

    odometryXEntry = createEntry("X", 0);
    odometryYEntry = createEntry("Y", 1);
    odometryAngleEntry = createEntry("Angle", 2);

    // This is an example of how you can use a lambda instead of the NetworkTableEntry method
    tab.addNumber(
        "Rotation Speed rad/s",
        () -> {
          ChassisSpeeds speeds;
          speeds = this.chassisSpeeds;
          if (speeds == null) return 0.0;

          return speeds.omegaRadiansPerSecond;
        });
  }

  /** Return the current pose estimation of the robot */
  public Pose2d getPose() {
    return robotPose;
  }

  /** Return the kinematics object, for constructing a trajectory */
  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }

  /**
   * Tells the subsystem to drive, and puts the state machine in drive mode
   *
   * @param chassisSpeeds the speed of the chassis desired
   */
  public void drive(ChassisSpeeds chassisSpeeds) {
    this.mode = Modes.DRIVE;
    this.chassisSpeeds = chassisSpeeds;
  }

  public void resetGyroAngle(Rotation2d angle) {
    // FEATURE: make work for resetting to non-zero angles
    navx.reset();
  }

  /** Sets the gyro angle to zero, resetting the forward direction */
  public void zeroGyroscope() {
    resetGyroAngle(Rotation2d.fromDegrees(0.0));
  }

  public Rotation2d getGyroscopeRotation() {
    if (navx.isMagnetometerCalibrated()) {
      // We will only get valid fused headings if the magnetometer is calibrated
      return Rotation2d.fromDegrees(navx.getFusedHeading());
    }

    // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes
    // the angle increase.
    return Rotation2d.fromDegrees(360.0 - navx.getYaw());
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
    angle = getGyroscopeRotation();

    this.robotPose = swerveOdometry.updateWithTime(time, angle, moduleStates);
  }

  /**
   * gets the current mode of the drivebase subsystem state machine
   *
   * @return the current mode
   */
  public Modes getMode() {
    return mode;
  }

  /**
   * Angles the swerve modules in a cross shape, to make the robot hard to push. This function sets
   * the state machine to defense mode, so it only needs to be called once
   */
  public void setDefenseMode() {
    mode = Modes.DEFENSE;
  }

  private void drivePeriodic() {
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

    // sets swerve module speeds and angles, for each swerve module, using kinematics
    for (int i = 0; i < swerveModules.length; i++) {
      swerveModules[i].set(
          states[i].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
          states[i].angle.getRadians());
    }
  }

  @SuppressWarnings("java:S1121")
  private void defensePeriodic() {
    int angle = 45;
    for (SwerveModule module : swerveModules) {
      // the *= -1 operation multiplies the current variable by -1, stores it, and also returns
      // the value. We can use this to alternate between 45 and -45 for each module.
      module.set(0, angle *= -1);
    }
  }

  /**
   * Based on the current Mode of the drivebase, write the relevant periodic method.
   *
   * @param mode The desired mode to write (should call getMode)
   */
  public void updateModules(Modes mode) {
    switch (mode) {
      case DRIVE:
        drivePeriodic();
        break;
      case DEFENSE:
        defensePeriodic();
        break;
    }
  }

  /** For use in #periodic, to calculate the timestamp between motor writes */
  private double lastTimestamp = 0.0;

  @Override
  public void periodic() {
    /* Calculate time since last run and update odometry accordingly */
    final double timestamp = Timer.getFPGATimestamp();
    final double dt = timestamp - lastTimestamp;
    lastTimestamp = timestamp;
    updateOdometry(timestamp);

    /* get the current set-points for the drivetrain */
    Modes currentMode = getMode();
    Pose2d pose = getPose();

    /*
     * See if there is a new drive signal from the trajectory follower object.
     * An Optional means that this value might be "present" or not exist (be null),
     * but provides somewhat more convenient semantics for checking if there is a
     * value or not without a great risk of causing an Exception.
     */
    Optional<ChassisSpeeds> trajectorySpeeds = follower.update(pose, timestamp, dt);

    /* If there is a trajectory signal, overwrite the current chassis speeds setpoint to that trajectory value*/
    if (trajectorySpeeds.isPresent()) {
      this.chassisSpeeds = trajectorySpeeds.get();
    }

    /* Write outputs, corresponding to our current Mode of operation */
    updateModules(currentMode);

    /* Update logging values on Shuffleboard widgets */
    odometryXEntry.setDouble(pose.getX());
    odometryYEntry.setDouble(pose.getY());
    odometryAngleEntry.setDouble(pose.getRotation().getDegrees());
  }
}
