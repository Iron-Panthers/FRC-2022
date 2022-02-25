// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
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
  private TalonFX armMotorOne;
  private TalonFX armMotorTwo;
  private PIDController pidController;
  private double DesiredAngle = Constants.Arm.PID.ANGULAR_OFFSET;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    armMotorOne = new TalonFX(Constants.Arm.Ports.MOTOR_1_PORT); // FIX LATER
    armMotorTwo = new TalonFX(Constants.Arm.Ports.MOTOR_2_PORT); // FIX LATER

    pidController = new PIDController(0.01, 0, 0.01);
    pidController.setSetpoint(0.0);
    pidController.setTolerance(Constants.Arm.PID.ANGULAR_TOLERANCE);
  }

  public void setPower(
      double Power) { // We probably will not be using this too much, more focused into using
    // positional values
    armMotorOne.set(TalonFXControlMode.PercentOutput, Power);
    armMotorTwo.set(TalonFXControlMode.PercentOutput, -Power);
  }

  public void setAngle(double dAngle) {
    DesiredAngle = dAngle; // Set the setpoint of the PIDController
    pidController.setSetpoint(DesiredAngle);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // If we're not moving towards a point, we should probably reset the PIDController
    double currentAngle =
        armMotorOne.getSelectedSensorPosition() * Constants.Arm.PID.TICKS_TO_DEGREES;

    double rotationValue = pidController.calculate(currentAngle - DesiredAngle);

    rotationValue = MathUtil.clamp(rotationValue, -1, 1);

    setPower(rotationValue);
    // Util.relativeAngularDifference();
  }
}
