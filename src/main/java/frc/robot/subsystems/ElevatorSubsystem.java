// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
  /** follower */
  private TalonFX left_motor;
  /** leader */
  private TalonFX right_motor;

  // private DigitalInput bottomLimitSwitch;
  // private DigitalInput topLimitSwitch;

  /** 2048 to a rotation */
  private double totalMotorRotationTicks;
  /** total rotations as a double */
  private double motorRotations;
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

    // topLimitSwitch = new DigitalInput(Constants.Elevator.Ports.TOP_SWITCH);
    // bottomLimitSwitch = new DigitalInput(Constants.Elevator.Ports.BOTTOM_SWITCH);

    this.totalMotorRotationTicks = 0.0;
    this.currentHeight = 0.0;
    this.targetHeight = 0.0;

    right_motor.configFactoryDefault();
    left_motor.configFactoryDefault();

    right_motor.clearStickyFaults();
    left_motor.clearStickyFaults();

    right_motor.configForwardSoftLimitThreshold(
        -2048 * 3, 0); // this is the bottom limit, we stop three full rotation before bottoming out
    right_motor.configReverseSoftLimitThreshold(
        -2048 * 62, 0); // this is the top limit, we stop before running out

    // // left_motor.configForwardSoftLimitThreshold(0, 0);
    // // left_motor.configReverseSoftLimitThreshold(2048 * 2, 0);

    right_motor.configForwardSoftLimitEnable(true, 0);
    right_motor.configReverseSoftLimitEnable(true, 0);

    left_motor.configForwardSoftLimitEnable(true, 0);
    left_motor.configReverseSoftLimitEnable(true, 0);

    left_motor.setSelectedSensorPosition(0);
    right_motor.setSelectedSensorPosition(0);

    // make sure we hold our height when we get disabled
    right_motor.setNeutralMode(NeutralMode.Brake);
    left_motor.setNeutralMode(NeutralMode.Brake);

    left_motor.follow(right_motor);

    ElevatorTab.add(heightController);
    ElevatorTab.addNumber("height", () -> this.currentHeight);
    ElevatorTab.addNumber("target height", () -> this.targetHeight);
    ElevatorTab.addNumber("right motor sensor value", right_motor::getSelectedSensorPosition);
  }

  /**
   * Stores the target height for the elevator, to be reached with motor adjustment
   *
   * @param targetHeight Uses Inches
   */
  public void setTargetHeight(double targetHeight) {
    SmartDashboard.putNumber("set target height", targetHeight);
    this.targetHeight = targetHeight;
  }

  public void setPercent(double percent) {
    right_motor.set(TalonFXControlMode.PercentOutput, percent);
  }

  /**
   * Calculate current height of elevator
   *
   * @return current Height in inches
   */
  public double getHeight() {

    // if (bottomLimitSwitchPressed()) {
    // this.totalMotorRotationTicks = 0;
    // } else {
    this.totalMotorRotationTicks = right_motor.getSelectedSensorPosition();

    this.motorRotations = this.totalMotorRotationTicks / 2048; // There are 2048 units per rotation

    this.currentHeight =
        ((motorRotations / 12.75) // 12.75:1 gear ratio
            * (1.5 * Math.PI)); // Circumference of gear multiplied by rotations to get height

    return currentHeight;
  }

  public double getTargetHeight() {
    return targetHeight;
  }

  public boolean topLimitSwitchPressed() {

    // return topLimitSwitch.get();
    return false;
  }

  public boolean bottomLimitSwitchPressed() {

    // return bottomLimitSwitch.get();
    return false;
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
