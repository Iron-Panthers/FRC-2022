// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Intake.EjectRollers;
import frc.robot.Constants.Intake.IntakeRollers;
import frc.robot.Constants.Intake.Ports;
import frc.util.LazyTalonFX;

public class IntakeSubsystem extends SubsystemBase {

  // final should be used on these fields, but if we use final mockito cannot inject mocks - use
  // final when you can
  /** the lower motor, upper motor follows this one - address this motor */
  private LazyTalonFX lowerMotor;
  /** follows lower motor, only address lower motor */
  private LazyTalonFX upperMotor;
  /** the right eject motor, aligns balls and allows rejections */
  private LazyTalonFX rightEjectMotor;
  /** the left eject motor, aligns balls and allows rejections */
  private LazyTalonFX leftEjectMotor;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    lowerMotor = new LazyTalonFX(Ports.LOWER_MOTOR);
    upperMotor = new LazyTalonFX(Ports.UPPER_MOTOR);

    rightEjectMotor = new LazyTalonFX(Ports.RIGHT_EJECT_MOTOR);
    leftEjectMotor = new LazyTalonFX(Ports.LEFT_EJECT_MOTOR);

    upperMotor.follow(lowerMotor);
    leftEjectMotor.follow(rightEjectMotor);
  }

  /** the different modes the intake subsystem state machine can be in */
  public enum Modes {
    OFF,
    IDLING,
    INTAKE,
    OUTTAKE,
    EJECT
  }

  /** the current mode of the subsystem */
  private Modes mode = Modes.OFF;

  /** Get the current state machine mode. */
  public Modes getMode() {
    return mode;
  }

  /**
   * This command should only be called once, after a given mode is finished, or prematurely stopped
   *
   * <p>Puts the subsystem state machine into the next mode. Some modes, like idling, are resting
   * points, from which there are not "next" modes. For other modes, like intake, idling is the
   * logical progression, and "next" mode
   */
  public void nextMode() {
    switch (mode) {
      case IDLING:
      case OFF:
        // these modes are resting points, and do not have a next mode until user input is provided
        break;
      case INTAKE:
        // after intake, we should run the idling motor to align balls for shooting and outtake
        setMode(Modes.IDLING);
        break;
      case EJECT:
      case OUTTAKE:
        // after ejection and outtake, we should stop all motors, because there shouldn't still be
        // balls in the intake
        setMode(Modes.OFF);
        break;
    }
  }

  /**
   * Sets the current state machine mode
   *
   * <p>fixme: this should have checks, but currently doesn't
   */
  public void setMode(Modes mode) {
    this.mode = mode;
  }

  /** periodic helper method, easy way to turn a motor off, by setting to 0 percent output */
  private void stopMotor(LazyTalonFX motor) {
    motor.set(TalonFXControlMode.PercentOutput, 0);
  }

  /** periodic helper method to make intention more readable. Stops the intake motors */
  private void stopIntake() {
    upperMotor.follow(lowerMotor);
    stopMotor(lowerMotor);
  }

  /** periodic helper method to make intention more readable. Stops the idler motor */
  private void stopIdler() {
    leftEjectMotor.follow(rightEjectMotor);
    stopMotor(rightEjectMotor);
  }

  private void offModePeriodic() {
    stopIdler();
    stopIntake();
  }

  private void idlingModePeriodic() {
    stopIntake();

    leftEjectMotor.follow(rightEjectMotor);
    rightEjectMotor.set(TalonFXControlMode.PercentOutput, EjectRollers.IDLE);
  }

  private void intakeModePeriodic() {
    idlingModePeriodic();

    upperMotor.follow(lowerMotor);
    lowerMotor.set(TalonFXControlMode.PercentOutput, IntakeRollers.INTAKE);
  }

  private void outtakeModePeriodic() {
    leftEjectMotor.follow(rightEjectMotor);
    rightEjectMotor.set(TalonFXControlMode.PercentOutput, EjectRollers.IDLE);

    lowerMotor.set(TalonFXControlMode.PercentOutput, IntakeRollers.OUTTAKE_LOWER);
    upperMotor.set(TalonFXControlMode.PercentOutput, IntakeRollers.OUTTAKE_UPPER);
  }

  private void ejectModePeriodic() {
    upperMotor.follow(lowerMotor);
    lowerMotor.set(TalonFXControlMode.PercentOutput, IntakeRollers.INTAKE);

    leftEjectMotor.follow(rightEjectMotor);
    rightEjectMotor.set(TalonFXControlMode.PercentOutput, EjectRollers.EJECT);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    switch (mode) {
      case OFF:
        offModePeriodic();
        break;
      case IDLING:
        idlingModePeriodic();
        break;
      case INTAKE:
        intakeModePeriodic();
        break;
      case OUTTAKE:
        outtakeModePeriodic();
        break;
      case EJECT:
        ejectModePeriodic();
        break;
    }
  }
}
