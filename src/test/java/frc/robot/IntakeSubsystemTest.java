package frc.robot;

import static org.junit.jupiter.api.Assertions.assertNotSame;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.fail;
import static org.mockito.Mockito.description;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.verifyNoMoreInteractions;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import frc.RobotParamTest;
import frc.RobotTest;
import frc.robot.Constants.Intake.EjectRollers;
import frc.robot.Constants.Intake.IntakeRollers;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.Modes;
import frc.util.LazyTalonFX;
import java.util.stream.Stream;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.extension.ExtendWith;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;
import org.mockito.InjectMocks;
import org.mockito.Mock;
import org.mockito.MockitoAnnotations;
import org.mockito.junit.jupiter.MockitoExtension;

@ExtendWith(MockitoExtension.class)
public class IntakeSubsystemTest {

  private AutoCloseable closeable;

  @InjectMocks private IntakeSubsystem intakeSubsystem;

  @Mock private LazyTalonFX lowerMotor;
  @Mock private LazyTalonFX upperMotor;
  @Mock private LazyTalonFX rightEjectMotor;
  @Mock private LazyTalonFX leftEjectMotor;

  private static String[] motorNames = {
    "lowerMotor", "upperMotor", "rightEjectMotor", "leftEjectMotor"
  };
  /** [lowerMotor, upperMotor, rightEjectMotor, leftEjectMotor] */
  private LazyTalonFX[] motorArray;

  @BeforeEach
  public void setup() {
    closeable = MockitoAnnotations.openMocks(intakeSubsystem);
    motorArray = new LazyTalonFX[] {lowerMotor, upperMotor, rightEjectMotor, leftEjectMotor};
  }

  /**
   * call the periodic methods
   *
   * <p>we call tick to let the intake react to our input - periodic ticks need to occur for our
   * mocked motors to have updates
   */
  private void tick() {
    intakeSubsystem.periodic();
  }

  /**
   * call the periodic methods repeatedly
   *
   * <p>we call tick to let the intake react to our input - periodic ticks need to occur for our
   * mocked motors to have updates. The updates will happen for each tick, so using multiple ticks
   * can ensure behavior is consistent
   */
  private void tick(int amount) {
    for (int i = 0; i < amount; i++) intakeSubsystem.periodic();
  }

  @AfterEach
  public void shutdown() {
    try {
      closeable.close();
    } catch (Exception e) {
      fail(e);
    }
  }

  @RobotTest
  public void intakeDefaultsToOff() {
    assertNotSame(IntakeSubsystem.Modes.EJECT, intakeSubsystem.getMode());
    assertSame(IntakeSubsystem.Modes.OFF, intakeSubsystem.getMode());
  }

  @RobotTest
  public void intakeMotorsAndEjectSpunDuringIntake() {
    intakeSubsystem.setMode(Modes.INTAKE);
    tick();
    verify(lowerMotor).set(TalonFXControlMode.PercentOutput, IntakeRollers.INTAKE);
    // there is no need to test upper motor, it is already following lower motor
    verify(rightEjectMotor).set(TalonFXControlMode.PercentOutput, EjectRollers.IDLE);
  }

  @RobotTest
  public void intakeKeepsSpinning() {
    intakeSubsystem.setMode(Modes.INTAKE);
    int calls = 3;
    tick(calls);
    verify(lowerMotor, times(calls)).set(TalonFXControlMode.PercentOutput, IntakeRollers.INTAKE);
  }

  private static Stream<Arguments> nextModeProgressionProvider() {
    return Stream.of(
        // this set should stay the same
        Arguments.of(Modes.OFF, Modes.OFF),
        Arguments.of(Modes.IDLING, Modes.IDLING),
        // this set should go off
        Arguments.of(Modes.EJECT, Modes.OFF),
        Arguments.of(Modes.OUTTAKE, Modes.OFF),
        // after intake, mode should be idle to keep aligning balls
        Arguments.of(Modes.INTAKE, Modes.IDLING));
  }

  // these two annotations tell junit to call the test repeatedly with the stream of arguments from
  // the above function, each as its own unit test
  @RobotParamTest
  @MethodSource("nextModeProgressionProvider")
  public void nextModeSwitchesProperly(Modes fromMode, Modes targetMode) {
    intakeSubsystem.setMode(fromMode);
    intakeSubsystem.nextMode();
    assertSame(targetMode, intakeSubsystem.getMode());
    assertSame(targetMode, intakeSubsystem.getMode());
  }

  // mega test below, here be dragons.

  private static enum Commands {
    FOLLOW,
    PER
  }

  private static class MotorCommand {
    private double percent = 0;
    private int masterIndex = -1;
    private Commands command;

    public MotorCommand(double percent, int masterIndex, Commands command) {
      this.percent = percent;
      this.masterIndex = masterIndex;
      this.command = command;
    }

    public static MotorCommand percent(double percent) {
      return new MotorCommand(percent, -1, Commands.PER);
    }

    public static MotorCommand follow(int masterIndex) {
      return new MotorCommand(0, masterIndex, Commands.FOLLOW);
    }

    public Commands getCommand() {
      return command;
    }

    public double getPercent() {
      return percent;
    }

    public int getMasterIndex() {
      return masterIndex;
    }
  }

  /**
   * returns the arguments to validate motor commands for a given mode
   *
   * @param mode the mode that the percentages apply to
   * @param lowerIntakePercent lower Intake motor percent
   * @param upperIntakePercent upper Intake motor percent
   * @param rightEjectPercent right ejection motor percent
   * @param leftEjectPercent left ejection motor percent
   * @return <code>Arguments.of(Modes, MotorCommand[4])</code> to describe motor percents for a
   *     given mode
   */
  private static Arguments targetMotorPercents(
      Modes mode,
      double lowerIntakePercent,
      double upperIntakePercent,
      double rightEjectPercent,
      double leftEjectPercent) {
    return Arguments.of(
        mode,
        new MotorCommand[] {
          MotorCommand.percent(lowerIntakePercent),
          MotorCommand.percent(upperIntakePercent),
          MotorCommand.percent(rightEjectPercent),
          MotorCommand.percent(leftEjectPercent)
        });
  }

  /**
   * returns the arguments to validate motor commands for a given mode
   *
   * @param mode the mode that the percentages apply to
   * @param lowerIntakePercent lower Intake motor percent
   * @param upperIntakePercent upper Intake motor percent
   * @param rightEjectPercent right ejection motor percent
   * @param leftEjectPercent left ejection motor percent
   * @return <code>Arguments.of(Modes, MotorCommand[4])</code> to describe motor percents for a
   *     given mode
   */
  private static Arguments targetMotorPercents(
      Modes mode, double lowerIntakePercent, double rightEjectPercent) {
    return Arguments.of(
        mode,
        new MotorCommand[] {
          MotorCommand.percent(lowerIntakePercent),
          MotorCommand.follow(0),
          MotorCommand.percent(rightEjectPercent),
          MotorCommand.follow(2)
        });
  }

  private static Stream<Arguments> modeMotorStatesProvider() {
    return Stream.of(
        targetMotorPercents(Modes.OFF, 0, 0),
        targetMotorPercents(Modes.IDLING, 0, EjectRollers.IDLE),
        targetMotorPercents(Modes.INTAKE, IntakeRollers.INTAKE, EjectRollers.IDLE),
        targetMotorPercents(
            Modes.OUTTAKE,
            IntakeRollers.OUTTAKE_LOWER,
            IntakeRollers.OUTTAKE_UPPER,
            EjectRollers.IDLE,
            EjectRollers.IDLE),
        targetMotorPercents(
            Modes.EJECT, 0, IntakeRollers.INTAKE, EjectRollers.EJECT, EjectRollers.EJECT));
  }

  @RobotParamTest
  @MethodSource("modeMotorStatesProvider")
  public void motorStatesMatchConstantsForMode(Modes mode, MotorCommand[] motorCommandArray) {
    intakeSubsystem.setMode(mode);
    tick();
    for (int i = 0; i < motorArray.length; i++) {
      MotorCommand motorCommand = motorCommandArray[i];
      switch (motorCommand.getCommand()) {
        case FOLLOW:
          verify(
                  motorArray[i],
                  description(
                      String.format(
                          "mode %s should have %s following %s, once",
                          mode, motorNames[i], motorNames[motorCommand.getMasterIndex()])))
              .follow(motorArray[motorCommand.getMasterIndex()]);
          break;
        case PER:
          verify(
                  motorArray[i],
                  description(
                      String.format(
                          "mode %s should have %s running at %s percent, once",
                          mode, motorNames[i], motorCommand.getPercent())))
              .set(TalonFXControlMode.PercentOutput, motorCommand.getPercent());
          break;
      }
      verifyNoMoreInteractions(motorArray[i]);
    }
  }
}
