// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/*
For Engineering:
   Need from engineering in ArmSubsystem: Right Motor Port, Left Motor Port, each motor's power attributes (scroll down), and Encoder Port 1
   Need from engineering in RobotContainer (Arm Team): Angle values in terms of buttons and the arm

   Also we defined the following motors "armRightMotor" and "armLeftMotor" in terms of their position from the elevator.
   "armRightMotor" is the motor right to the elevator (assuming that on a plan view of the robot, the elevator is on the bottom),
   and "armLeftMotor" is the motor left of the elevator.
*/
public class ArmSubsystem extends SubsystemBase {

  private TalonFX leftMotor;
  private TalonFX rightMotor;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    leftMotor = new TalonFX(Constants.Arm.Ports.leftMotor);
    rightMotor = new TalonFX(Constants.Arm.Ports.rightMotor);

    leftMotor.setNeutralMode(NeutralMode.Brake);
    rightMotor.setNeutralMode(NeutralMode.Brake);

    rightMotor.follow(leftMotor);

    rightMotor.setInverted(InvertType.OpposeMaster);
  }

  public void setManualPower(double power) {
    leftMotor.set(TalonFXControlMode.PercentOutput, power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
