// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/*

    Todo:
        1. Make and initialize two motors *DONE*
        2. Define functions in this subsystem to move the arm a direction
          2a. Figure out how to do some cool PID stuff
        3. Make a command ArmCommand.java that has handles moving the arm
        4. Define certain buttons to call command functions that move our arm some amount of degrees
            4a. We should have encoders to be able to move by degrees somehow

 */

public class ArmSubsystem extends SubsystemBase {
  private TalonFX armMotorOne;
  private TalonFX armMotorTwo;
  private static PIDController pidController;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    armMotorOne = new TalonFX(Constants.Arm.MOTOR_1_PORT);//FIX LATER
    armMotorTwo = new TalonFX(Constants.Arm.MOTOR_2_PORT);//FIX LATER
    pidController = new PIDController(1.0, 1.0, 1.0);               // TODO: Look into having to edit the PID Values to be propert
  }

  public void setPower(double Power) {
    armMotorOne.set(TalonFXControlMode.PercentOutput, Power);
    armMotorTwo.set(TalonFXControlMode.PercentOutput, -Power);
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}