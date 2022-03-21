// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Intake.EjectRollers;
import frc.robot.Constants.Intake.IntakeRollers;
import frc.robot.Constants.Intake.Ports;
import frc.util.MacUtil;

public class IntakeSubsystem extends SubsystemBase {

  // final should be used on these fields, but if we use final mockito cannot inject mocks - use
  // final when you can
  /** the lower motor, upper motor follows this one - address this motor */
  private TalonFX lowerIntakeMotor;
  /** follows lower motor, only address lower motor */
  private TalonFX upperIntakeMotor;
  /** the right eject motor, aligns balls and allows rejections */
  private TalonFX rightEjectMotor;
  /** the left eject motor, aligns balls and allows rejections */
  private TalonFX leftEjectMotor;

  private void configStatusFramePeriodsAndBatteryComp(TalonFX talon) {
    talon.setStatusFramePeriod(1, 100);
    talon.setStatusFramePeriod(2, 100);
    talon.enableVoltageCompensation(true);
    talon.configVoltageCompSaturation(12);
  }

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    lowerIntakeMotor = new TalonFX(Ports.LOWER_MOTOR);
    if (MacUtil.IS_COMP_BOT) {
      lowerIntakeMotor.setInverted(false); // sin
    }
    upperIntakeMotor = new TalonFX(Ports.UPPER_MOTOR);

    rightEjectMotor = new TalonFX(Ports.RIGHT_EJECT_MOTOR);
    rightEjectMotor.setInverted(true);
    leftEjectMotor = new TalonFX(Ports.LEFT_EJECT_MOTOR);

    configStatusFramePeriodsAndBatteryComp(lowerIntakeMotor);
    configStatusFramePeriodsAndBatteryComp(upperIntakeMotor);
    configStatusFramePeriodsAndBatteryComp(rightEjectMotor);
    configStatusFramePeriodsAndBatteryComp(leftEjectMotor);
  }

  /** the different modes the intake subsystem state machine can be in */
  public enum Modes {
    OFF,
    IDLING,
    INTAKE,
    OUTTAKE,
    OUTTAKE_FAST,
    OUTTAKE_HIGH,
    EJECT_LEFT,
    EJECT_RIGHT,
    EJECT_ALL,
    ALIGN_INTERNAL,
  }

  /** the current mode of the subsystem */
  private Modes mode = Modes.OFF;

  /** Get the current state machine mode. */
  public Modes getMode() {
    return mode;
  }

  private long ejectionDebounceBaseTime = RobotController.getFPGATime();

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
        // setMode(Modes.IDLING);
        // break;
      case EJECT_LEFT:
      case EJECT_RIGHT:
      case EJECT_ALL:
      case ALIGN_INTERNAL:
      case OUTTAKE:
      case OUTTAKE_FAST:
      case OUTTAKE_HIGH:
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
    if (mode == Modes.OUTTAKE_HIGH) {
      ejectionDebounceBaseTime = RobotController.getFPGATime();
    }
    this.mode = mode;
  }

  /** periodic helper method, easy way to turn a motor off, by setting to 0 percent output */
  private void stopMotor(TalonFX motor) {
    motor.set(TalonFXControlMode.PercentOutput, 0);
  }

  /** periodic helper method to make intention more readable. Stops the intake motors */
  private void stopIntakeRollers() {
    stopMotor(upperIntakeMotor);
    stopMotor(lowerIntakeMotor);
  }

  /** periodic helper method to make intention more readable. Stops the idler motor */
  private void stopEjectRollers() {
    stopMotor(leftEjectMotor);
    stopMotor(rightEjectMotor);
  }

  private void runEjectRollers(double percent) {
    runLeftEjectMotor(percent);
    runRightEjectRoller(percent);
  }

  private void runLeftEjectMotor(double percent) {
    leftEjectMotor.set(TalonFXControlMode.PercentOutput, percent);
  }

  private void runRightEjectRoller(double percent) {
    rightEjectMotor.set(TalonFXControlMode.PercentOutput, percent);
  }

  private void runIntakeRollers(double percent) {
    upperIntakeMotor.set(TalonFXControlMode.PercentOutput, percent);
    lowerIntakeMotor.set(TalonFXControlMode.PercentOutput, percent);
  }

  private void feedBallsViaIntakeForEject() {
    stopMotor(lowerIntakeMotor);
    upperIntakeMotor.set(TalonFXControlMode.PercentOutput, IntakeRollers.INTAKE);
  }

  private void offModePeriodic() {
    stopEjectRollers();
    stopIntakeRollers();
  }

  private void idlingModePeriodic() {
    stopIntakeRollers();
    runEjectRollers(EjectRollers.IDLE);
  }

  private void intakeModePeriodic() {
    runEjectRollers(EjectRollers.IDLE);
    runIntakeRollers(IntakeRollers.INTAKE);
  }

  private void outtakeModePeriodic() {
    runEjectRollers(EjectRollers.IDLE);

    lowerIntakeMotor.set(TalonFXControlMode.PercentOutput, IntakeRollers.OUTTAKE_LOWER);
    upperIntakeMotor.set(TalonFXControlMode.PercentOutput, IntakeRollers.OUTTAKE_UPPER);
  }

  private void outtakeFastModePeriodic() {
    runEjectRollers(EjectRollers.IDLE);

    lowerIntakeMotor.set(TalonFXControlMode.PercentOutput, IntakeRollers.OUTTAKE_LOWER_FAST);
    upperIntakeMotor.set(TalonFXControlMode.PercentOutput, IntakeRollers.OUTTAKE_UPPER_FAST);
  }

  private void outtakeHighModePeriodic() {
    long curTime = RobotController.getFPGATime();
    // 250,000 nanoseconds = 0.25s
    if (curTime - ejectionDebounceBaseTime >= 250_000) {
      runLeftEjectMotor(EjectRollers.IDLE);
    }
    // 500,000 nanoseconds= 0.5s
    if (curTime - ejectionDebounceBaseTime >= 500_000) {
      runRightEjectRoller(EjectRollers.IDLE);
    }

    lowerIntakeMotor.set(TalonFXControlMode.PercentOutput, IntakeRollers.OUTTAKE_LOWER_HIGH);
    upperIntakeMotor.set(TalonFXControlMode.PercentOutput, IntakeRollers.OUTTAKE_UPPER_HIGH);
  }

  private void ejectLeftModePeriodic() {
    runLeftEjectMotor(EjectRollers.EJECT);
    runRightEjectRoller(EjectRollers.IDLE);

    feedBallsViaIntakeForEject();
  }

  private void ejectRightModePeriodic() {
    runLeftEjectMotor(EjectRollers.IDLE);
    runRightEjectRoller(EjectRollers.EJECT);

    feedBallsViaIntakeForEject();
  }

  private void ejectAllModePeriodic() {
    runEjectRollers(EjectRollers.EJECT);

    feedBallsViaIntakeForEject();
  }

  private void alignInternalPeriodic() {
    runEjectRollers(EjectRollers.ALIGN_INTERNAL);
    stopMotor(lowerIntakeMotor);
    upperIntakeMotor.set(TalonFXControlMode.PercentOutput, IntakeRollers.ALIGN_INTERNAL);
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
      case OUTTAKE_FAST:
        outtakeFastModePeriodic();
        break;
      case OUTTAKE_HIGH:
        outtakeHighModePeriodic();
        break;
      case EJECT_LEFT:
        ejectLeftModePeriodic();
        break;
      case EJECT_RIGHT:
        ejectRightModePeriodic();
        break;
      case EJECT_ALL:
        ejectAllModePeriodic();
        break;
      case ALIGN_INTERNAL:
        alignInternalPeriodic();
        break;
    }
  }
}
