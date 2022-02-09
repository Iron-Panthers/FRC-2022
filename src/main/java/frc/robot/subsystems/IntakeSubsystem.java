// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.Intake;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  private TalonFX lowerMotor;
  private TalonFX upperMotor;
  private TalonFX idlerMotor;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    lowerMotor = new TalonFX(Intake.LOWER_MOTOR);
    upperMotor = new TalonFX(Intake.UPPER_MOTOR);
    idlerMotor = new TalonFX(Intake.IDLER_MOTOR);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
