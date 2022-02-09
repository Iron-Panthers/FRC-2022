// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.Intake;

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
    lowerMotor = new TalonFX(Intake.LOWER_MOTOR);
    upperMotor = new TalonFX(Intake.UPPER_MOTOR);
    idlerMotor = new TalonFX(Intake.IDLER_MOTOR);
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

  private void offModePeriodic() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    switch (mode) {
    }
  }
}
