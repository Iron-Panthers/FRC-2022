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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private final TalonFX armRightMotor;
  private final TalonFX armLeftMotor;
  private final PIDController pidController;
  private final CANCoder armEncoder;

  private double desiredAngle = 0; // Figure out what to change to fit the size-parameter

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    armRightMotor = new TalonFX(Constants.Arm.Ports.RightMotorPort); // FIX ME - Identify motor port
    armLeftMotor = new TalonFX(Constants.Arm.Ports.LeftMotorPort); // FIX Me

    pidController = new PIDController(0.01, 0, 0.01);
    pidController.setTolerance(Constants.Arm.PID.ANGULAR_TOLERANCE);

    armEncoder =
        new CANCoder(
            Constants.Arm.Ports.ENCODER_PORT); // FIX ME: we will need to figure out the real value
    armEncoder.configFactoryDefault();
    armEncoder.configSensorInitializationStrategy(
        SensorInitializationStrategy.BootToAbsolutePosition);
    // armEncoder.configMagnetOffset(Constants.Arm.ANGULAR_OFFSET);
  }

  /**
   * We probably will not be using this too much, more focused into using positional values Need
   * engineering to figure out if the right or left motor uses positive or negative power,
   * respectively
   */
  public void setPower(double power) {

    armRightMotor.set(TalonFXControlMode.PercentOutput, power); // Engineering FIX ME
    armLeftMotor.set(TalonFXControlMode.PercentOutput, -power);
  }

  // Sets the goal of the pid controller
  public void setAngle(double desiredAngle) {
    this.desiredAngle = desiredAngle; // Set the setpoint of the PIDController
  }

  public void stopMotor() {
    setPower(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double currentAngle = armEncoder.getAbsolutePosition();

    SmartDashboard.putNumber("currentAngle", currentAngle);

    // double output = controller.calculate(measurement (what is actually there), desired value
    // (where we want it to be))
    // -> PID math gibberish -> the output we want to write to our motor(s)

    final double output = pidController.calculate(currentAngle, desiredAngle);
    final double clampedOutput = MathUtil.clamp(output, -1, 1);

    // Add the gravity offset as a function of cosine
    final double gOffset = Math.cos(currentAngle) * 0.05;

    setPower(clampedOutput + gOffset);
    // Util.relativeAngularDifference();
  }
}
