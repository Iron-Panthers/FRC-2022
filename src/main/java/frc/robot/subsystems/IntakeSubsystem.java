// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Intake.Ports;
import frc.util.MacUtil;

public class IntakeSubsystem extends SubsystemBase {

  private TalonFX intakeLower;
  private TalonFX intakeUpper;
  private TalonFX ejectLeft;
  private TalonFX ejectRight;

  public IntakeSubsystem(){

    intakeLower = new TalonFX(Constants.Intake.Ports.intakeLower);
    intakeUpper = new TalonFX(Constants.Intake.Ports.intakeUpper);
    ejectLeft = new TalonFX(Constants.Intake.Ports.ejectLeft);
    ejectRight = new TalonFX(Constants.Intake.Ports.ejectRight);

    ejectLeft.setInverted(true);

  }
 
  public void setIntake (double power){
    intakeLower.set(TalonFXControlMode.PercentOutput, power);
    intakeUpper.set(TalonFXControlMode.PercentOutput, power);
  }

  public void setEjectLeft(double power){
    ejectLeft.set(TalonFXControlMode.PercentOutput, power);
  }

  public void setEjectRight(double power){
    ejectRight.set(TalonFXControlMode.PercentOutput, power);
  }



}

//During outtake first spin upper intake and eject towards each other so it sucks the balls back. Then spin 1st eject and shoot, then shoot 2nd eject and shoot