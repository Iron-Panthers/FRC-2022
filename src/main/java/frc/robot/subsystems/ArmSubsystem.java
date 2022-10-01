// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Arm.Setpoints;
import frc.robot.Constants.Elevator;

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

  private ElevatorSubsystem elevatorSubsystem;

  private double desiredAngle = Arm.Setpoints.OUTTAKE_HIGH_POSITION;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem(ElevatorSubsystem elevatorSubsystem) {

    this.elevatorSubsystem = elevatorSubsystem;

    armRightMotor = new TalonFX(Arm.Ports.RIGHT_MOTOR_PORT);
    armLeftMotor = new TalonFX(Arm.Ports.LEFT_MOTOR_PORT);

    armRightMotor.setNeutralMode(NeutralMode.Brake);
    armLeftMotor.setNeutralMode(NeutralMode.Brake);

    armLeftMotor.follow(armRightMotor);
    armLeftMotor.setInverted(InvertType.OpposeMaster);

    armRightMotor.configOpenloopRamp(0.2);

    armRightMotor.setStatusFramePeriod(1, 100);
    armRightMotor.setStatusFramePeriod(2, 100);

    armLeftMotor.setStatusFramePeriod(1, 500);
    armLeftMotor.setStatusFramePeriod(2, 500);

    var config =
        new StatorCurrentLimitConfiguration(
            true /*enable*/,
            50 /* current limit */,
            5 /* threshold */,
            .1 /*time in seconds to trip*/);

    armRightMotor.configStatorCurrentLimit(config);
    armLeftMotor.configStatorCurrentLimit(config);

    pidController = new PIDController(0.007, 0, 0.00015);
    pidController.setTolerance(Arm.PID.ANGULAR_TOLERANCE);

    Shuffleboard.getTab("arm").add(pidController);

    armEncoder =
        new CANCoder(Arm.Ports.ENCODER_PORT); // FIX ME: we will need to figure out the real value
    armEncoder.configFactoryDefault();
    armEncoder.configSensorInitializationStrategy(
        SensorInitializationStrategy.BootToAbsolutePosition);
    armEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    armEncoder.configMagnetOffset(Arm.ANGULAR_OFFSET);
    armEncoder.configSensorDirection(true);
    armEncoder.setPositionToAbsolute(10); // ms

    ShuffleboardLayout tab =
        Shuffleboard.getTab("DriverView")
            .getLayout("arm", BuiltInLayouts.kList)
            .withSize(2, 2)
            .withPosition(16, 1);
    tab.addNumber("target angle", this::getTargetAngle);
    tab.addNumber("actual angle", this::getAngle);
  }

  public double getAngle() {
    return armEncoder.getAbsolutePosition();
  }

  public double getTargetAngle() {
    return desiredAngle;
  }

  private void setPercentOutput(double power) {
    armRightMotor.set(TalonFXControlMode.PercentOutput, power);
  }

  private void setCurrentOutput(double current) {
    armRightMotor.set(TalonFXControlMode.Current, current);
  }

  // Sets the goal of the pid controller
  public void setAngle(double desiredAngle) {
    this.desiredAngle = desiredAngle; // Set the setpoint of the PIDController
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    final double currentAngle = getAngle();

    // make target angle safe lol
    desiredAngle =
        MathUtil.clamp(
            desiredAngle,
            Setpoints.INTAKE_POSITION,
            elevatorSubsystem.getHeight() >= Elevator.ENGAGED_HEIGHT
                ? Setpoints.CLIMB_POSITION
                : Setpoints.MAX_HEIGHT);

    // SmartDashboard.putNumber("current angle", currentAngle);
    // SmartDashboard.putNumber("desired angle", desiredAngle);
    // SmartDashboard.putNumber("constant setpoint MaxHeight - ", currentAngle -
    // Setpoints.MAX_HEIGHT);

    // double output = controller.calculate(measurement (what is actually there), desired value
    // (where we want it to be))
    // -> PID math gibberish -> the output we want to write to our motor(s)

    final double pidOutput = pidController.calculate(currentAngle, desiredAngle);

    // SmartDashboard.putNumber("output", pidOutput);

    final double clampedOutput =
        MathUtil.clamp(
            pidOutput, -1 + Arm.GRAVITY_CONTROL_PERCENT, 1 - Arm.GRAVITY_CONTROL_PERCENT);

    // SmartDashboard.putNumber("clamped output", clampedOutput);

    // Add the gravity offset as a function of cosine
    final double gravityOffset =
        Math.cos(Math.toRadians(currentAngle)) * Arm.GRAVITY_CONTROL_PERCENT;

    SmartDashboard.putNumber("gravityOffset", gravityOffset);
    SmartDashboard.putNumber("cos angle", Math.cos(Math.toRadians(currentAngle)));

    // runs the arm at a constant voltage if the arm is past the hardstop limit
    if ((desiredAngle - Arm.Setpoints.OUTTAKE_HIGH_POSITION < Arm.Hardstop.ERROR_MARGIN)
        && (currentAngle - Arm.Setpoints.OUTTAKE_HIGH_POSITION < Arm.Hardstop.ERROR_MARGIN)) {
      setPercentOutput(Arm.Hardstop.HOLD_VOLTAGE);
    } else {
      setCurrentOutput(MathUtil.clamp(clampedOutput + gravityOffset, -.5, .5));
    }

    // SmartDashboard.putNumber("motor percent", motorPercent);

  }
}
