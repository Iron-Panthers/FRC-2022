// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
 
package frc.robot.subsystems;
 
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Elevator;
 
public class ElevatorSubsystem extends SubsystemBase {
  private final TalonFX left = new TalonFX(Constants.Elevator.Ports.LEFT_MOTOR);
  private final TalonFX right = new TalonFX(Constants.Elevator.Ports.RIGHT_MOTOR);
  private final DigitalInput bottomLimitSwitch =
      new DigitalInput(Elevator.Ports.BOTTOM_SWITCH); // do we need to put constants?
  private final DigitalInput topLimitSwitch =
      new DigitalInput(Elevator.Ports.TOP_SWITCH); // do we need to put constants?
 
  private double totalMotorRotationTicks;
  private double motorRotations;
  private double absoluteHeight;
  private double targetHeight;
  private double motorPower;
  private final PIDController heightController = new PIDController(0.04, 0, 0);
 
  // clockwise moves up
  // counterclockwise moves down
 
  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
 
    this.totalMotorRotationTicks = 0.0;
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
   * Stores the target height for the elevator, to be reached with motor adjustment
   *
   * @param targetHeight Uses Inches
   */
  public void setTargetHeight(Double targetHeight) {
    this.targetHeight = targetHeight;
  }
 
  /**
   * Calculate absolute height of elevator
   *
   * @return Absolute Height in inches
   */
  public double getHeight() {
 
    /* Lexie's code
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
    */
 
    if (bottomLimitSwitchPressed()) {
      this.totalMotorRotationTicks = 0;
    } else {
      this.totalMotorRotationTicks = right.getSelectedSensorPosition();
    }
 
    this.motorRotations = this.totalMotorRotationTicks / 4096; // There are 4096 units per rotation
 
    this.absoluteHeight =
        ((motorRotations / 12.75) // 12.75:1 gear ratio
            * (1.5 * Math.PI)); // Circumference of gear multiplied by rotations to get height
 
    return absoluteHeight;
  }
 
  public static boolean epsilonEquals(double a, double b, double epsilon) {
    return (a - epsilon <= b) && (a + epsilon >= b);
  }
 
  public void lock() {
 
    right.setNeutralMode(NeutralMode.Brake);
  }
 
  public boolean topLimitSwitchPressed() {
 
    return topLimitSwitch.get();
  }
 
  public boolean bottomLimitSwitchPressed() {
 
    return bottomLimitSwitch.get();
  }
 
  @Override
  public void periodic() {
    double currentHeight = getHeight();
    this.motorPower = heightController.calculate(getHeight(), targetHeight);
    right.set(TalonFXControlMode.PercentOutput, motorPower);
    if (epsilonEquals(currentHeight, targetHeight, .1)){
      lock();
    }
    if (topLimitSwitchPressed() || bottomLimitSwitchPressed()) {
      lock();
    } // FIX ME (Isaac, we don't know if it'll be stuck at the limit switch)
    // This method will be called once per scheduler run
  }
}
 
// 12.75 full motor rotations = 1.5pi inches of height
// 12.75:1 gear ratio
// talonfx to the entire thing
// Big gear is 2.6 inches
// 1.5 sprocket diameter