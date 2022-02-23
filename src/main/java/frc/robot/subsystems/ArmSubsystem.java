// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import java.lang.FdLibm.Pow;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*

    Todo:
        1. Make and initialize two motors
      
        2. Define functions in this subsystem to move the arm a direction
        3. Make a command ArmCommand.java that has handles moving the arm
        4. Define certain buttons to call command functions that move our arm some amount of degrees
            4a. We should have encoders to be able to move by degrees somehow

 */

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  public static WPI_TalonFX armMotorOne;
  public static WPI_TalonFX armMotorTwo;

  public ArmSubsystem() {
    armMotorOne = new WPI_TalonFX(0);//FIX LATER
    armMotorTwo = new WPI_TalonFX(0);//FIX LATER
  }

public static void setPower(double Power){
  armMotorOne.set(TalonFXControlMode.PercentOutput, Power);
  armMotorTwo.set(TalonFXControlMode.PercentOutput, Power);
}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
