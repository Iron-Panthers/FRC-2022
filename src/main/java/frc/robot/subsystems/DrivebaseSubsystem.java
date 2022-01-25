// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.Drive.*;

import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.Util;

public class DrivebaseSubsystem extends SubsystemBase {
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

  private final AHRS navx = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX connected over MXP

  private final SwerveModule frontLeftModule;
  private final SwerveModule frontRightModule;
  private final SwerveModule backLeftModule;
  private final SwerveModule backRightModule;

  private final SwerveModule[] swerveModules;

  private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(); // defaults to zeros

  /**
   * initialize a falcon with a shuffleboard tab, and mk4 default gear ratio
   *
   * @param title the shuffleboard title
   * @param pos the shuffleboard x position, which is <b>multiplied by 2</b>
   * @param drive the drive motor port const
   * @param steer the steer motor port const
   * @param encoder the encoder port const
   * @param offset the steer offset const, found experimentally
   * @return an sds swerve module object
   */
  private SwerveModule createModule(
      String title, int pos, int drive, int steer, int encoder, double offset) {
    ShuffleboardTab tab = Shuffleboard.getTab("Drivebase");

    return Mk4SwerveModuleHelper.createFalcon500(
        tab.getLayout(title, BuiltInLayouts.kList).withSize(2, 4).withPosition(pos * 2, 0),
        Mk4SwerveModuleHelper.GearRatio.L2,
        drive,
        steer,
        encoder,
        offset);
  }

  public enum Modes {
    DRIVE,
    DEFENSE,
  }

  private Modes mode = Modes.DRIVE;

  /** Creates a new DrivebaseSubsystem. */
  public DrivebaseSubsystem() {
    frontRightModule =
        createModule(
            "Front Right Module #1",
            1,
            Modules.FrontRight.DRIVE_MOTOR,
            Modules.FrontRight.STEER_MOTOR,
            Modules.FrontRight.STEER_ENCODER,
            Modules.FrontRight.STEER_OFFSET);

    frontLeftModule =
        createModule(
            "Front Left Module #2",
            0,
            Modules.FrontLeft.DRIVE_MOTOR,
            Modules.FrontLeft.STEER_MOTOR,
            Modules.FrontLeft.STEER_ENCODER,
            Modules.FrontLeft.STEER_OFFSET);

    backLeftModule =
        createModule(
            "Back Left Module #3",
            2,
            Modules.BackLeft.DRIVE_MOTOR,
            Modules.BackLeft.STEER_MOTOR,
            Modules.BackLeft.STEER_ENCODER,
            Modules.BackLeft.STEER_OFFSET);

    backRightModule =
        createModule(
            "Back Right Module #4",
            3,
            Modules.BackRight.DRIVE_MOTOR,
            Modules.BackRight.STEER_MOTOR,
            Modules.BackRight.STEER_ENCODER,
            Modules.BackRight.STEER_OFFSET);

    swerveModules =
        new SwerveModule[] {frontRightModule, frontLeftModule, backLeftModule, backRightModule};
  }

  /** Sets the gyro angle to zero, resetting the forward direction */
  public void zeroGyroscope() {
    navx.zeroYaw();
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

  /**
   * Tells the subsystem to drive, and puts the state machine in drive mode
   *
   * @param chassisSpeeds the speed of the chassis desired
   */
  public void drive(ChassisSpeeds chassisSpeeds) {
    this.chassisSpeeds = chassisSpeeds;
    mode = Modes.DEFENSE;
  }

  public Modes getMode() {
    return mode;
  }

  /** Angles the swerve modules in a cross shape, to make the robot hard to push. */
  public void setDefense() {
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

  private void defensePeriodic() {
    int angle = 90 + 45;
    for (SwerveModule module : swerveModules) {
      // if the module is close to aligned, we are done with it
      // FIXME find this epsilon experimentally
      if (Util.epsilonEquals(module.getSteerAngle(), angle, 1e-2)) continue;
      module.set(0, angle);
      angle += 90;
    }
  }

  @Override
  public void periodic() {
    switch (mode) {
      case DRIVE:
        drivePeriodic();
        break;
      case DEFENSE:
        defensePeriodic();
        break;
    }
  }
}
