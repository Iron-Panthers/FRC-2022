// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

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
  private final TalonFX armMotorOne;
  private final TalonFX armMotorTwo;
  private final PIDController pidController;

  private final CANCoder armEncoder;

  private double desiredAngle = Constants.Arm.PID.ANGULAR_OFFSET;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    armMotorOne = new TalonFX(Constants.Arm.Ports.MOTOR_1_PORT); // FIX LATER
    armMotorTwo = new TalonFX(Constants.Arm.Ports.MOTOR_2_PORT); // FIX LATER

    pidController = new PIDController(0.01, 0, 0.01);
    pidController.setTolerance(Constants.Arm.PID.ANGULAR_TOLERANCE);

    armEncoder = new CANCoder(200); // FIXME: we will need to figure out the real value
    armEncoder.configFactoryDefault();
    armEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
  }

  public void setPower(double power) {
    // We probably will not be using this too much, more focused into using positional values
    armMotorOne.set(TalonFXControlMode.PercentOutput, power);
    armMotorTwo.set(TalonFXControlMode.PercentOutput, -power);
  }

  // Sets the goal of the pid controller
  public void setAngle(double dAngle) {
    desiredAngle = dAngle; // Set the setpoint of the PIDController
  }

  public void stopMotor(){
    setPower(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double currentAngle = armEncoder.getAbsolutePosition();

    // double output = controller.calculate(measurement (what is actually there), desired value (where we want it to be))
    // -> PID math gibberish -> the output we want to write to our motor(s)

    final double output = pidController.calculate(currentAngle, desiredAngle);
    final double clampedOutput = MathUtil.clamp(output, -1, 1);

    setPower(clampedOutput);
    // Util.relativeAngularDifference();
  }
}
