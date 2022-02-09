// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.Intake;
import static frc.robot.Constants.Intake.Ports;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  /** the lower motor, upper motor follows this one - address this motor */
  private TalonFX lowerMotor;
  /** follows lower motor, only address lower motor */
  private TalonFX upperMotor;
  /** the idiler motor, aligns balls and allows rejections */
  private TalonFX idlerMotor;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    lowerMotor = new TalonFX(Ports.LOWER_MOTOR);
    upperMotor = new TalonFX(Ports.UPPER_MOTOR);
    idlerMotor = new TalonFX(Ports.IDLER_MOTOR);
    upperMotor.follow(lowerMotor);
  }

  public enum Modes {
    OFF,
    IDLING,
    INTAKE,
    OUTTAKE,
    EJECT
  }

  private Modes mode = Modes.OFF;

  /** Get the current state machine mode. */
  public Modes getMode() {
    return mode;
  }

  /**
   * Sets the current state machine mode
   *
   * <p>fixme: this should have checks, but currently doesn't
   */
  public void setMode(Modes mode) {
    this.mode = mode;
  }

  private void stopMotor(TalonFX motor) {
    motor.set(TalonFXControlMode.PercentOutput, 0);
  }

  private void offModePeriodic() {
    stopMotor(lowerMotor);
    stopMotor(idlerMotor);
  }

  private void idlingModePeriodic() {
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
}
