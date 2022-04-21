// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
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

  private final PIDController heightController;

  // clockwise moves up
  // counterclockwise moves down

  private final ShuffleboardTab ElevatorTab = Shuffleboard.getTab("ElevatorTab");

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    heightController = new PIDController(0.000075, 0, 0);

    left_motor = new TalonFX(Constants.Elevator.Ports.LEFT_MOTOR);
    right_motor = new TalonFX(Constants.Elevator.Ports.RIGHT_MOTOR);

    currentHeight = 0.0;
    targetHeight = 0.0;

    right_motor.configFactoryDefault();
    left_motor.configFactoryDefault();

    right_motor.clearStickyFaults();
    left_motor.clearStickyFaults();

    right_motor.configForwardSoftLimitThreshold(
        0, 0); // this is the bottom limit, we stop AT the bottom
    right_motor.configReverseSoftLimitThreshold(
        -heightToTicks(24), 0); // this is the top limit, we stop at the very top

    right_motor.configForwardSoftLimitEnable(true, 0);
    right_motor.configReverseSoftLimitEnable(true, 0);

    right_motor.configOpenloopRamp(.5);

    setSensorHeight(0);

    // make sure we hold our height when we get disabled
    right_motor.setNeutralMode(NeutralMode.Brake);
    left_motor.setNeutralMode(NeutralMode.Brake);

    left_motor.follow(right_motor);

    ElevatorTab.add(heightController);
    ElevatorTab.addNumber("height", () -> this.currentHeight);
    ElevatorTab.addNumber("target height", () -> this.targetHeight);
    ElevatorTab.addNumber("right motor sensor value", this::getHeight);

    ShuffleboardLayout driverView =
        Shuffleboard.getTab("DriverView")
            .getLayout("elevator", BuiltInLayouts.kList)
            .withSize(2, 2)
            .withPosition(16, 3);
    driverView.addNumber("elevator % height", () -> getHeight() / -heightToTicks(24));
    driverView.addNumber("elevator height inches", this::getHeight);
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
    if (
    // going down block
    (percent >= 0 /* going down */
            && height <= SlowZone.LOWER_THRESHHOLD /* inside the lower slow zone */)
        ||
        // going up block
        (percent <= 0 /* going up */
            && height >= SlowZone.UPPER_THRESHHOLD /* inside the upper slow zone */)) {

      // modify the output by the slowzone modifier

      return percent * SlowZone.SLOWZONE_MODIFIER;
    }
    // not in slow zone conditions, proceed as normal
    return percent;
  }

  /**
   * Stores the target height for the elevator, to be reached with motor adjustment
   *
   * @param targetHeight Uses Inches
   */
  public void setTargetHeight(double targetHeight) {
    this.targetHeight = targetHeight;
  }

  private void setSensorHeight(double ticks) {
    left_motor.setSelectedSensorPosition(ticks);
    right_motor.setSelectedSensorPosition(ticks);
  }

  public void setPercent(double percent) {
    right_motor.set(
        TalonFXControlMode.PercentOutput, applySlowZoneToPercent(percent * Elevator.MAX_PERCENT));
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

  @Override
  public void periodic() {

    // currentHeight = getHeight();
    // double motorPower = heightController.calculate(getHeight(), targetHeight);
    // right_motor.set(TalonFXControlMode.PercentOutput, motorPower);

    // // if (topLimitSwitchPressed() || bottomLimitSwitchPressed()) {
    // //   lock();
    // // } // FIXME (Isaac, we don't know if it'll be stuck at the limit switch)

    // // // shuffleboard?
    // SmartDashboard.putNumber("Height", currentHeight);
    // SmartDashboard.putNumber("Target Height", targetHeight);

    // SmartDashboard.putNumber("Motor Power", motorPower);
    // SmartDashboard.putNumber("Top Limit Switch Pressed", topLimitSwitchPressed());
    // SmartDashboard.putNumber("Bottom Limit Switch Pressed", bottomLimitSwitchPressed());
    // This method will be called once per scheduler run
  }
}

// 12.75 full motor rotations = 1.5pi inches of height
// 12.75:1 gear ratio
// talonfx to the entire thing
// Big gear is 2.6 inches
// 1.5 sprocket diameter
