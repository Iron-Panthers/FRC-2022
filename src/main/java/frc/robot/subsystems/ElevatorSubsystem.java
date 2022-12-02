// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Elevator;
import frc.robot.Constants.Elevator.SlowZone;

public class ElevatorSubsystem extends SubsystemBase {
  /** follower */
  private TalonFX left_motor;
  /** leader */
  private TalonFX right_motor;

  /** Elevator's current height in inches */
  private double currentHeight;

  /** desired height in inches */
  private double targetHeight;

  private double percentControl;

  private double percentOutput;

  private final ProfiledPIDController heightController;

  // clockwise moves up
  // counterclockwise moves down

  private final ShuffleboardTab ElevatorTab = Shuffleboard.getTab("ElevatorTab");

  /** The modes of the elevator subsystem */
  public enum Modes {
    PERCENT_CONTROL,
    POSITION_CONTROL,
    NEUTRAL,
    COAST
  }

  // the current mode
  private Modes mode = Modes.PERCENT_CONTROL;

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    heightController =
        new ProfiledPIDController(
            0.3, 0, 0, new TrapezoidProfile.Constraints(8, 5)); // idk abt constraint values
    heightController.setTolerance(.35);

    left_motor = new TalonFX(Constants.Elevator.Ports.LEFT_MOTOR);
    right_motor = new TalonFX(Constants.Elevator.Ports.RIGHT_MOTOR);

    currentHeight = 0.0;
    targetHeight = 0.0;

    right_motor.configFactoryDefault();
    left_motor.configFactoryDefault();

    right_motor.clearStickyFaults();
    left_motor.clearStickyFaults();

    right_motor.configForwardSoftLimitThreshold(
        -heightToTicks(Elevator.MIN_HEIGHT), 0); // this is the bottom limit, we stop AT the bottom
    right_motor.configReverseSoftLimitThreshold(
        -heightToTicks(Elevator.MAX_HEIGHT), 0); // this is the top limit, we stop at the very top

    right_motor.configForwardSoftLimitEnable(true, 0);
    right_motor.configReverseSoftLimitEnable(true, 0);

    right_motor.configOpenloopRamp(.5);

    setSensorHeight(0);

    // make sure we hold our height when we get disabled
    right_motor.setNeutralMode(NeutralMode.Brake);
    left_motor.setNeutralMode(NeutralMode.Brake);
    left_motor.follow(right_motor);

    ElevatorTab.add("pid", heightController);
    ElevatorTab.addNumber("height", () -> this.currentHeight);
    ElevatorTab.addNumber("target height", () -> this.targetHeight);
    ElevatorTab.addNumber("right motor sensor value", this::getHeight);
    ElevatorTab.addString("mode", () -> this.getMode().toString());
    ElevatorTab.addNumber("percent output", this::getPercentOutput);
    ElevatorTab.addNumber("elevator height ticks", right_motor::getSelectedSensorPosition);
  }

  public static double heightToTicks(double height) {
    return height * ((Elevator.GEAR_RATIO * Elevator.TICKS) / (Elevator.GEAR_CIRCUMFERENCE));
  }

  public static double ticksToHeight(double ticks) {
    return (ticks * Elevator.GEAR_CIRCUMFERENCE) / (Elevator.TICKS * Elevator.GEAR_RATIO);
  }

  /**
   * If the elevator is inside the slow mode, and the percent would move the elevator in, this code
   * returns a scaled down version of the percent it was passed. otherwise it returns the full
   * amount.
   *
   * @param percent the percent to apply slow mode to
   * @return the new percent, possibly scaled down to accommodate slow mode
   */
  private double applySlowZoneToPercent(double percent) {
    final double height = getHeight();
    double clampPercent = MathUtil.clamp(percent, -1, 1) * Elevator.MAX_PERCENT;
    if (
    // going down block
    (clampPercent >= 0 /* going down */
            && height <= SlowZone.LOWER_THRESHHOLD /* inside the lower slow zone */)
        ||
        // going up block
        (clampPercent <= 0 /* going up */
            && height >= SlowZone.UPPER_THRESHHOLD /* inside the upper slow zone */)) {

      // modify the output by the slowzone modifier

      return clampPercent * SlowZone.SLOWZONE_MODIFIER;
    }
    // not in slow zone conditions, proceed as normal
    return clampPercent;
  }

  private double applySlowZoneToPID(double percent) {
    final double height = getHeight();
    double clampPercent = MathUtil.clamp(percent, -1, 1);
    if (
    // going down block
    (percent >= 0 /* going down */
            && height <= SlowZone.LOWER_THRESHHOLD /* inside the lower slow zone */)
        ||
        // going up block
        (percent <= 0 /* going up */
            && height >= SlowZone.UPPER_THRESHHOLD /* inside the upper slow zone */)) {

      // modify the output by the slowzone modifier

      return MathUtil.clamp(clampPercent, -SlowZone.SLOWZONE_MODIFIER, SlowZone.SLOWZONE_MODIFIER);

    } else {
      return clampPercent;
    }
  }
  /**
   * Stores the target height for the elevator, to be reached with motor adjustment
   *
   * @param targetHeight Uses Inches
   */
  private void setSensorHeight(double ticks) {
    left_motor.setSelectedSensorPosition(ticks);
    right_motor.setSelectedSensorPosition(ticks);
  }

  public void setNeutral() {
    this.mode = Modes.NEUTRAL;
    right_motor.setNeutralMode(NeutralMode.Brake);
    left_motor.setNeutralMode(NeutralMode.Brake);
  }

  public void setCoast() {
    this.mode = Modes.COAST;
    right_motor.setNeutralMode(NeutralMode.Coast);
    left_motor.setNeutralMode(NeutralMode.Coast);
  }

  public void setTargetHeight(double targetHeight) {
    this.mode = Modes.POSITION_CONTROL;
    this.targetHeight = targetHeight;
    heightController.reset(getHeight());
  }

  public void setPercent(double percent) {
    this.mode = Modes.PERCENT_CONTROL;
    this.percentControl = percent;
  }

  /**
   * Calculate current height of elevator
   *
   * @return current Height in inches
   */
  public double getHeight() {
    // // we take the negative, because running it in reverse goes up
    return ticksToHeight(-right_motor.getSelectedSensorPosition());
  }

  /** does nothing rn */
  public double getTargetHeight() {
    return targetHeight;
  }

  public Modes getMode() {
    return mode;
  }

  public double getPercentOutput() {
    return percentOutput;
  }

  public boolean atTarget() {
    return heightController.atSetpoint();
  }

  /**
   * based on the current mode, returns the correct percent value. Default returns 0
   *
   * @return the percent to set the motors to without adjusting for max speed or safety clamping
   */
  private double getPercentControl() {
    Modes currentMode = getMode();
    currentHeight = getHeight();
    targetHeight = getTargetHeight();
    switch (currentMode) {
      case PERCENT_CONTROL:
        percentOutput = applySlowZoneToPercent(percentControl);
        return percentOutput;
      case POSITION_CONTROL:
        percentOutput =
            MathUtil.clamp(-heightController.calculate(currentHeight, targetHeight), -.5, .5);
        return percentOutput;
      default:
        return 0;
    }
  }

  @Override
  public void periodic() {

    // based on mode
    // set percent to member or pid output

    right_motor.set(TalonFXControlMode.PercentOutput, getPercentControl());
  }
}

// 12.75 full motor rotations = 1.5pi inches of height
// 12.75:1 gear ratio
// Big gear is 2.6 inches
// 1.5 sprocket diameter
