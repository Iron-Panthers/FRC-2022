// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Intake.EjectRollers;
import frc.robot.Constants.Intake.IntakeRollers;
import frc.robot.Constants.Intake.ModeWaits;
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
    talon.setStatusFramePeriod(2, 20);
    talon.configVoltageCompSaturation(11);
    talon.enableVoltageCompensation(true);
    talon.setNeutralMode(NeutralMode.Coast);
  }

  private void applyPID(TalonFX motor) {
    motor.config_kP(0, .07);
    motor.config_kI(0, 0);
    motor.config_kD(0, 0);
    motor.config_kF(0, .05);
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

    applyPID(upperIntakeMotor);
    applyPID(lowerIntakeMotor);

    var tab = Shuffleboard.getTab("intake");

    tab.addNumber("RIGHT roller", rightEjectMotor::getSelectedSensorVelocity);
    tab.addNumber("left roller", leftEjectMotor::getSelectedSensorVelocity);

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
    INTAKE_FORCEFUL,
    ALIGN_LOW,
    OUTTAKE_LOW_LEFT,
    OUTTAKE_LOW_ALL,
    ALIGN_HIGH,
    OUTTAKE_HIGH_LEFT,
    OUTTAKE_HIGH_ALL,
    EJECT_LEFT,
    EJECT_RIGHT,
    EJECT_ALL,
  }

  /** the current mode of the subsystem */
  private Modes mode = Modes.OFF;

  /** timestamp in fpga seconds of mode transition */
  private double timeOfModeTransition = Timer.getFPGATimestamp();

  /**
   * this variable represents if automatic mode transitions are locked. this is used to hold in a
   * mode. this value should never explicitly be set
   */
  private boolean modeLocked = false;

  /** Get the current state machine mode. */
  public Modes getMode() {
    return mode;
  }

  /** Gets the boolean that determines if the mode is locked, preventing transition */
  public boolean getModeLocked() {
    return modeLocked;
  }

  /**
   * Time in seconds since mode has changed, good for modes that only run for a certain amount of
   * time.
   *
   * @return time in seconds since mode change
   */
  public double timeSinceModeTransition() {
    return Timer.getFPGATimestamp() - timeOfModeTransition;
  }

  /**
   * This function is called in periodic to increment the state machine
   *
   * <p>Puts the subsystem state machine into the next mode. Some modes, like idling, are resting
   * points, from which there are not "next" modes. For other modes, like intake, idling is the
   * logical progression, and "next" mode
   */
  private void nextModePeriodic() {
    // if our mode is locked, we should not increment the mode
    if (modeLocked) return;

    switch (mode) {
      case OFF:
        // these modes are resting points, and do not have a next mode until user input is provided
        break;

        // high shot block
      case ALIGN_HIGH:
        // when our time is up, we transition to high left, otherwise we wait at this mode
        if (timeSinceModeTransition() >= ModeWaits.High.ALIGN_TO_LEFT) {
          setMode(Modes.OUTTAKE_HIGH_LEFT);
        }
        break;
      case OUTTAKE_HIGH_LEFT:
        if (timeSinceModeTransition() >= ModeWaits.High.LEFT_TO_ALL) {
          setMode(Modes.OUTTAKE_HIGH_ALL);
        }
        break;
      case OUTTAKE_HIGH_ALL:
        if (timeSinceModeTransition() >= ModeWaits.High.ALL_TO_OFF) {
          setMode(Modes.OFF);
        }
        break;
        // end high shot block

        // intake block
      case INTAKE:
        // intake forceful has the same next mode as intake
      case INTAKE_FORCEFUL:
        // after intake, we should run the idling motor for a time to align balls for shooting and
        // outtake
        setMode(Modes.IDLING);
        break;
      case IDLING:
        if (timeSinceModeTransition() >= ModeWaits.IntakeWaits.IDLE_TO_OFF) {
          setMode(Modes.OFF);
        }
        break;

        // end intake block

        // outtake block
      case ALIGN_LOW:
        if (timeSinceModeTransition() >= ModeWaits.Outtake.ALIGN_TO_LEFT) {
          setMode(Modes.OUTTAKE_LOW_LEFT);
        }
        break;
      case OUTTAKE_LOW_LEFT:
        if (timeSinceModeTransition() >= ModeWaits.Outtake.LEFT_TO_ALL) {
          setMode(Modes.OUTTAKE_LOW_ALL);
        }
        break;
      case OUTTAKE_LOW_ALL:
        if (timeSinceModeTransition() >= ModeWaits.Outtake.ALL_TO_OFF) {
          setMode(Modes.OFF);
        }
        break;
        // end outtake block

        // this set of modes should go to off
      case EJECT_LEFT:
      case EJECT_RIGHT:
      case EJECT_ALL:
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
    timeOfModeTransition = Timer.getFPGATimestamp();
    this.mode = mode;
    modeLocked = false;
  }

  /** sets mode, and locks it to prevent state transitions */
  public void setLockMode(Modes mode) {
    setMode(mode);
    modeLocked = true;
  }

  /** unlocks mode transitions, so that they can occur */
  public void unlockMode() {
    modeLocked = false;
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

  private void runEjectRollersPercent(double percent) {
    runLeftEjectRollerPercent(percent);
    runRightEjectRollerPercent(percent);
  }

  private void runLeftEjectRollerPercent(double percent) {
    leftEjectMotor.set(TalonFXControlMode.PercentOutput, percent);
  }

  private void runRightEjectRollerPercent(double percent) {
    rightEjectMotor.set(TalonFXControlMode.PercentOutput, percent);
  }

  private void runIntakeRollersPercent(double percent) {
    upperIntakeMotor.set(TalonFXControlMode.PercentOutput, percent);
    lowerIntakeMotor.set(TalonFXControlMode.PercentOutput, percent);
  }

  private void runEjectRollersVelocity(double velocity) {
    runLeftEjectRollerVelocity(velocity);
    runRightEjectRollerVelocity(velocity);
  }

  private void runLeftEjectRollerVelocity(double velocity) {
    leftEjectMotor.set(TalonFXControlMode.Velocity, velocity);
  }

  private void runRightEjectRollerVelocity(double velocity) {
    rightEjectMotor.set(TalonFXControlMode.Velocity, velocity);
  }

  private void runIntakeRollersVelocity(double velocity) {
    upperIntakeMotor.set(TalonFXControlMode.Velocity, velocity);
    lowerIntakeMotor.set(TalonFXControlMode.Velocity, velocity);
  }

  private void feedBallsViaIntakeForEject() {
    stopMotor(lowerIntakeMotor);
    upperIntakeMotor.set(TalonFXControlMode.PercentOutput, IntakeRollers.INTAKE);
  }

  // periodic section

  private void offModePeriodic() {
    stopEjectRollers();
    stopIntakeRollers();
  }

  private void idlingModePeriodic() {
    stopMotor(lowerIntakeMotor);
    upperIntakeMotor.set(TalonFXControlMode.PercentOutput, IntakeRollers.INTAKE);

    runEjectRollersPercent(EjectRollers.IDLE);
  }

  private void intakeModePeriodic() {
    runEjectRollersPercent(EjectRollers.IDLE);
    runIntakeRollersPercent(IntakeRollers.INTAKE);
  }

  private void intakeForcefulModePeriodic() {
    runEjectRollersPercent(EjectRollers.IDLE);
    runIntakeRollersPercent(IntakeRollers.INTAKE_FORCEFUL);
  }

  private void alignLowPeriodic() {
    runEjectRollersPercent(EjectRollers.ALIGN_INTERNAL);
    stopMotor(lowerIntakeMotor);
    upperIntakeMotor.set(TalonFXControlMode.PercentOutput, IntakeRollers.ALIGN_INTERNAL);
  }

  private void outtakeLowLeftModePeriodic() {
    runLeftEjectRollerPercent(EjectRollers.FEED_LOW);
    stopMotor(rightEjectMotor);

    lowerIntakeMotor.set(TalonFXControlMode.Velocity, IntakeRollers.OUTTAKE_LOWER_LOW);
    upperIntakeMotor.set(TalonFXControlMode.Velocity, IntakeRollers.OUTTAKE_UPPER_LOW);
  }

  private void outtakeLowAllModePeriodic() {
    runEjectRollersPercent(EjectRollers.FEED_LOW);

    lowerIntakeMotor.set(TalonFXControlMode.Velocity, IntakeRollers.OUTTAKE_LOWER_LOW);
    upperIntakeMotor.set(TalonFXControlMode.Velocity, IntakeRollers.OUTTAKE_UPPER_LOW);
  }

  private void alignHighPeriodic() {
    runEjectRollersPercent(EjectRollers.ALIGN_INTERNAL);
    stopMotor(lowerIntakeMotor);
    upperIntakeMotor.set(TalonFXControlMode.PercentOutput, IntakeRollers.ALIGN_INTERNAL);
  }

  private void outtakeHighLeftModePeriodic() {
    runLeftEjectRollerPercent(EjectRollers.FEED_HIGH);
    stopMotor(rightEjectMotor);

    lowerIntakeMotor.set(TalonFXControlMode.Velocity, IntakeRollers.OUTTAKE_LOWER_HIGH);
    upperIntakeMotor.set(TalonFXControlMode.Velocity, IntakeRollers.OUTTAKE_UPPER_HIGH);
  }

  private void outtakeHighAllModePeriodic() {
    runEjectRollersPercent(EjectRollers.FEED_HIGH);

    lowerIntakeMotor.set(TalonFXControlMode.Velocity, IntakeRollers.OUTTAKE_LOWER_HIGH);
    upperIntakeMotor.set(TalonFXControlMode.Velocity, IntakeRollers.OUTTAKE_UPPER_HIGH);
  }

  private void ejectLeftModePeriodic() {
    runLeftEjectRollerPercent(EjectRollers.EJECT);
    runRightEjectRollerPercent(EjectRollers.IDLE);

    feedBallsViaIntakeForEject();
  }

  private void ejectRightModePeriodic() {
    runLeftEjectRollerPercent(EjectRollers.IDLE);
    runRightEjectRollerPercent(EjectRollers.EJECT);

    feedBallsViaIntakeForEject();
  }

  private void ejectAllModePeriodic() {
    runEjectRollersPercent(EjectRollers.EJECT);

    feedBallsViaIntakeForEject();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    nextModePeriodic();

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
      case INTAKE_FORCEFUL:
        intakeForcefulModePeriodic();
        break;
      case ALIGN_LOW:
        alignLowPeriodic();
        break;
      case OUTTAKE_LOW_LEFT:
        outtakeLowLeftModePeriodic();
        break;
      case OUTTAKE_LOW_ALL:
        outtakeLowAllModePeriodic();
        break;
      case ALIGN_HIGH:
        alignHighPeriodic();
        break;
      case OUTTAKE_HIGH_LEFT:
        outtakeHighLeftModePeriodic();
        break;
      case OUTTAKE_HIGH_ALL:
        outtakeHighAllModePeriodic();
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
    }
  }
}
