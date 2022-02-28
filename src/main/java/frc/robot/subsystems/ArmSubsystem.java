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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Arm;
import frc.util.Util;

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

  private double desiredAngle = Arm.Setpoints.MAX_HEIGHT;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    armRightMotor = new TalonFX(Arm.Ports.RightMotorPort); // FIX ME - Identify motor port
    armLeftMotor = new TalonFX(Arm.Ports.LeftMotorPort); // FIX Me

    pidController = new PIDController(0.0001, 0, 0);
    pidController.setTolerance(Arm.PID.ANGULAR_TOLERANCE);
    pidController.setSetpoint(0);
    Shuffleboard.getTab("arm").add(pidController);

    armEncoder =
        new CANCoder(Arm.Ports.ENCODER_PORT); // FIX ME: we will need to figure out the real value
    armEncoder.configFactoryDefault();
    armEncoder.configSensorInitializationStrategy(
        SensorInitializationStrategy.BootToAbsolutePosition);
  }

  public double getAngle() {
    return Util.relativeAngularDifference(armEncoder.getPosition(), Arm.ANGULAR_OFFSET);
  }

  private void setPercentOutput(double power) {
    armRightMotor.set(TalonFXControlMode.PercentOutput, power);
    armLeftMotor.set(TalonFXControlMode.PercentOutput, -power);
  }

  // Sets the goal of the pid controller
  public void setAngle(double desiredAngle) {
    this.desiredAngle = desiredAngle; // Set the setpoint of the PIDController
  }

  public void stopMotor() {
    setPercentOutput(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double currentAngle = getAngle();

    SmartDashboard.putNumber("current angle", currentAngle);
    SmartDashboard.putNumber("desired angle", desiredAngle);

    // double output = controller.calculate(measurement (what is actually there), desired value
    // (where we want it to be))
    // -> PID math gibberish -> the output we want to write to our motor(s)

    final double angularDiff =
        Math.abs(currentAngle - desiredAngle) * (desiredAngle < currentAngle ? -1 : 1);

    SmartDashboard.putNumber("angular diff", angularDiff);

    final double output = pidController.calculate(angularDiff);

    SmartDashboard.putNumber("output", output);

    final double clampedOutput =
        MathUtil.clamp(output, -1 + Arm.GRAVITY_CONTROL_PERCENT, 1 - Arm.GRAVITY_CONTROL_PERCENT);

    SmartDashboard.putNumber("clamped output", clampedOutput);

    // Add the gravity offset as a function of cosine
    final double gOffset = Math.cos(Math.toRadians(currentAngle)) * Arm.GRAVITY_CONTROL_PERCENT;

    SmartDashboard.putNumber("gOffset", gOffset);

    final double motorPercent = clampedOutput + gOffset;

    SmartDashboard.putNumber("motor percent", motorPercent);

    setPercentOutput(MathUtil.clamp(motorPercent, -.3, .3));
  }
}
