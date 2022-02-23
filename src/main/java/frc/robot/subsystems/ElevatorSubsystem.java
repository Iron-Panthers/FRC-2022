// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
  private final TalonFX left = new TalonFX(Constants.Elevator.ELEVATOR_MOTOR);
  private final TalonFX right = new TalonFX(Constants.Elevator.ELEVATOR_MOTOR_2);
  private final PIDController heightController = new PIDController(0.04, 0, 0);

  // clockwise moves up
  // counterclockwise moves down

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {

    // Configure the right motor
    right.setInverted(true);

    // Follow with the left motor and copy any important things
    left.follow(right);
    left.setInverted(TalonFXInvertType.FollowMaster);
  }

  public void setMotorPower(Double power) {
    right.set(TalonFXControlMode.PercentOutput, power);
  }

  public double getMotorPosition() {
    return right.getSelectedSensorPosition();
  }

  public void setHeight(Double goalPosition) {
    right.set(
        TalonFXControlMode.PercentOutput,
        heightController.calculate(getMotorPosition(), goalPosition));
  }

  public void lock() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
