// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
  private final TalonFX left = new TalonFX(Constants.Elevator.Ports.LEFT_MOTOR);
  private final TalonFX right = new TalonFX(Constants.Elevator.Ports.RIGHT_MOTOR);
  private double totalMotorRotation;
  private double absoluteHeight;
  private double pastMotorRotation;
  private double currentMotorRotation;
  private double targetHeight;
  private double motorPower;
  private final PIDController heightController = new PIDController(0.04, 0, 0);

  // clockwise moves up
  // counterclockwise moves down

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {

    this.totalMotorRotation = 0.0;
    this.absoluteHeight = 0.0;
    // Configure the right motor
    right.setInverted(true);

    // Follow with the left motor and copy any important things
    left.follow(right);
    left.setInverted(TalonFXInvertType.FollowMaster);
  }

  public void setMotorPower(Double power) {
    right.set(TalonFXControlMode.PercentOutput, power);
  }

  /**
   * stores the target height for the elevator, to be reached with motor adjustment
   *
   * @param targetHeight Uses Inches
   */
  public void setHeight(Double targetHeight) {
    this.targetHeight = targetHeight;
  }

  public double getHeightInInches() {

    this.pastMotorRotation = this.currentMotorRotation;
    this.currentMotorRotation = right.getSelectedSensorPosition();

    /* this was the old way, which was too idealisitic
        if (currentMotorRotation == 0 && pastMotorRotation == 4096) {
          this.totalMotorRotation += 4096;
        } else if (currentMotorRotation == 4096 && pastMotorRotation == 0) {
          this.totalMotorRotation -= 4096;
        } else {

        }
    */

    if (motorPower >= 0) {
      if (currentMotorRotation < pastMotorRotation) {
        this.totalMotorRotation += 4096;
      }
      this.totalMotorRotation += (currentMotorRotation - pastMotorRotation);
    } else if (motorPower < 0) {
      if (currentMotorRotation > pastMotorRotation) {
        this.totalMotorRotation -= 4096;
      }
      this.totalMotorRotation += (currentMotorRotation - pastMotorRotation);
    }

    this.absoluteHeight =
        (((this.totalMotorRotation + this.currentMotorRotation) / 4096) / 12.75)
            * (1.5 * Math.PI); // number of motor rotations converted to height in inches

    return absoluteHeight;
  }

  public void lock() {

    right.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    this.motorPower = heightController.calculate(getHeightInInches(), targetHeight);
    right.set(TalonFXControlMode.PercentOutput, motorPower);
    // This method will be called once per scheduler run
  }
}

// 12.75 full motor rotations = 1.5pi inches of height

// 12.75:1 gear ratio
// talonfx to the entire thing
// Big gear is 2.6 inches
// 1.5 sprocket diameter
