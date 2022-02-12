// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.Intake;
import static frc.robot.Constants.Intake.Ports;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase implements AutoCloseable {

  /** the lower motor, upper motor follows this one - address this motor */
  private TalonFX lowerMotor;
  /** follows lower motor, only address lower motor */
  private TalonFX upperMotor;
  /** the idler motor, aligns balls and allows rejections */
  private TalonFX idlerMotor;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    lowerMotor = new TalonFX(Ports.LOWER_MOTOR);
    upperMotor = new TalonFX(Ports.UPPER_MOTOR);
    idlerMotor = new TalonFX(Ports.IDLER_MOTOR);
    upperMotor.follow(lowerMotor);
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
  private void stopMotor(TalonFX motor) {
    motor.set(TalonFXControlMode.PercentOutput, 0);
  }

  private void offModePeriodic() {
    stopMotor(lowerMotor);
    stopMotor(idlerMotor);
  }

  private void idlingModePeriodic() {
    stopMotor(lowerMotor);
    idlerMotor.set(TalonFXControlMode.PercentOutput, Intake.IDLER_PERCENT);
  }

  private void intakeModePeriodic() {
    idlingModePeriodic();
    lowerMotor.set(TalonFXControlMode.PercentOutput, Intake.INTAKE_PERCENT);
  }

  private void outtakeModePeriodic() {
    idlingModePeriodic();
    lowerMotor.set(TalonFXControlMode.PercentOutput, Intake.OUTTAKE_PERCENT);
  }

  private void ejectModePeriodic() {
    stopMotor(lowerMotor);
    idlerMotor.set(TalonFXControlMode.PercentOutput, Intake.EJECT_PERCENT);
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

  @Override
  public void close() {
    // these error codes are ignored. this may be undesirable in the future.
    lowerMotor.DestroyObject();
    upperMotor.DestroyObject();
    idlerMotor.DestroyObject();
  }
}
