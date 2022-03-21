package frc.robot;

import static org.junit.jupiter.api.Assertions.assertNotSame;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.fail;
import static org.mockito.Mockito.description;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.verifyNoMoreInteractions;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.RobotParamTest;
import frc.RobotTest;
import frc.robot.Constants.Intake.EjectRollers;
import frc.robot.Constants.Intake.IntakeRollers;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.Modes;
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

  @Mock private TalonFX lowerIntakeMotor;
  @Mock private TalonFX upperIntakeMotor;
  @Mock private TalonFX rightEjectMotor;
  @Mock private TalonFX leftEjectMotor;

  private static String[] motorNames = {
    "lowerIntakeMotor", "upperIntakeMotor", "rightEjectMotor", "leftEjectMotor"
  };
  /** [lowerIntakeMotor, upperIntakeMotor, rightEjectMotor, leftEjectMotor] */
  private TalonFX[] motorArray;

  @BeforeEach
  public void setup() {
    closeable = MockitoAnnotations.openMocks(intakeSubsystem);
    motorArray =
        new TalonFX[] {lowerIntakeMotor, upperIntakeMotor, rightEjectMotor, leftEjectMotor};
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
    assertNotSame(IntakeSubsystem.Modes.EJECT_LEFT, intakeSubsystem.getMode());
    assertNotSame(IntakeSubsystem.Modes.EJECT_RIGHT, intakeSubsystem.getMode());
    assertSame(IntakeSubsystem.Modes.OFF, intakeSubsystem.getMode());
  }

  @RobotTest
  public void intakeMotorsAndEjectSpunDuringIntake() {
    intakeSubsystem.setMode(Modes.INTAKE);
    tick();
    verify(lowerIntakeMotor).set(TalonFXControlMode.PercentOutput, IntakeRollers.INTAKE);
    // there is no need to test upper motor, it is already following lower motor
    verify(rightEjectMotor).set(TalonFXControlMode.PercentOutput, EjectRollers.IDLE);
  }

  @RobotTest
  public void intakeKeepsSpinning() {
    intakeSubsystem.setMode(Modes.INTAKE);
    int calls = 3;
    tick(calls);
    verify(lowerIntakeMotor, times(calls))
        .set(TalonFXControlMode.PercentOutput, IntakeRollers.INTAKE);
  }

  private static Stream<Arguments> nextModeProgressionProvider() {
    return Stream.of(
        // this set should stay the same
        Arguments.of(Modes.OFF, Modes.OFF),
        Arguments.of(Modes.IDLING, Modes.IDLING),
        // this set should go off
        Arguments.of(Modes.EJECT_LEFT, Modes.OFF),
        Arguments.of(Modes.EJECT_RIGHT, Modes.OFF),
        Arguments.of(Modes.OUTTAKE, Modes.OFF),
        Arguments.of(Modes.OUTTAKE_FAST, Modes.OFF)
        // Arguments.of(Modes.OUTTAKE_HIGH, Modes.OFF),
        // after intake, mode should be idle to keep aligning balls (NOT TRUE ANYMORE)
        // Arguments.of(Modes.INTAKE, Modes.OFF)
        );
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

  // here be dragons...

  /**
   * returns the arguments to validate motor commands for a given mode
   *
   * @param mode the mode that the percentages apply to
   * @param intakePercent Intake motor percent
   * @param ejectPercent Ejection motor percent
   * @return <code>Arguments.of(Modes, MotorCommand[4])</code> to describe motor percents for a
   *     given mode
   */
  private static Arguments targetMotorPercents(
      Modes mode, double intakePercent, double ejectPercent) {
    return Arguments.of(
        mode, new double[] {intakePercent, intakePercent, ejectPercent, ejectPercent});
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
        new double[] {lowerIntakePercent, upperIntakePercent, rightEjectPercent, leftEjectPercent});
  }

  private static Stream<Arguments> modeMotorStatesProvider() {
    return Stream.of(
        // off, turn everything off
        targetMotorPercents(Modes.OFF, 0 /*intake*/, 0 /*eject*/),

        // idling, run only the eject motors to align balls
        targetMotorPercents(Modes.IDLING, 0 /*intake*/, EjectRollers.IDLE /*eject*/),

        // intake, run the eject for alignment and the intake to hoover balls
        targetMotorPercents(
            Modes.INTAKE, IntakeRollers.INTAKE /*intake*/, EjectRollers.IDLE /*eject*/),

        // outtake, run the eject to feed the balls in and run the intake in reverse
        targetMotorPercents(
            Modes.OUTTAKE,
            IntakeRollers.OUTTAKE_LOWER /*lower intake*/,
            IntakeRollers.OUTTAKE_UPPER /*upper intake*/,
            EjectRollers.IDLE /*right eject*/,
            EjectRollers.IDLE /*left eject*/),

        // outtake, run the eject to feed the balls in and run the intake in reverse
        targetMotorPercents(
            Modes.OUTTAKE_FAST,
            IntakeRollers.OUTTAKE_LOWER_FAST /*lower intake*/,
            IntakeRollers.OUTTAKE_UPPER_FAST /*upper intake*/,
            EjectRollers.IDLE /*right eject*/,
            EjectRollers.IDLE /*left eject*/),

        // // outtake for high shot, run the eject to feed the balls in and run the intake in
        // reverse
        // targetMotorPercents(
        //     Modes.OUTTAKE_HIGH,
        //     IntakeRollers.OUTTAKE_LOWER_HIGH /*lower intake*/,
        //     IntakeRollers.OUTTAKE_UPPER_HIGH /*upper intake*/,
        //     EjectRollers.IDLE /*right eject*/,
        //     EjectRollers.IDLE /*left eject*/),

        // eject left, run the upper intake to feed the balls into the ejection rollers to expel
        targetMotorPercents(
            Modes.EJECT_LEFT,
            0 /*lower intake*/,
            IntakeRollers.INTAKE /*upper intake*/,
            EjectRollers.IDLE /*right eject*/,
            EjectRollers.EJECT /*left eject*/),

        // same but eject right
        targetMotorPercents(
            Modes.EJECT_RIGHT,
            0 /*lower intake*/,
            IntakeRollers.INTAKE /*upper intake*/,
            EjectRollers.EJECT /*right eject*/,
            EjectRollers.IDLE /*left eject*/),

        // run both ejection rollers
        targetMotorPercents(
            Modes.EJECT_ALL,
            0 /*lower intake*/,
            IntakeRollers.INTAKE /*upper intake*/,
            EjectRollers.EJECT /*right eject*/,
            EjectRollers.EJECT /*left eject*/));
  }

  @RobotParamTest
  @MethodSource("modeMotorStatesProvider")
  public void motorStatesMatchConstantsForMode(Modes mode, double[] motorPercentArray) {
    intakeSubsystem.setMode(mode);
    tick();
    for (int i = 0; i < motorArray.length; i++) {
      double percent = motorPercentArray[i];
      verify(
              motorArray[i],
              description(
                  String.format(
                      "mode %s should have %s running at %s percent, once",
                      mode, motorNames[i], percent)))
          .set(TalonFXControlMode.PercentOutput, percent);

      // remove this line to get more useful failures for the above tests
      verifyNoMoreInteractions(motorArray[i]);
    }
  }
}
