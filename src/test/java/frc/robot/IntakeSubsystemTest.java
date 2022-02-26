package frc.robot;

import static org.junit.jupiter.api.Assertions.assertNotSame;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.fail;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;

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

  /**
   * contains [lowerMotor, rightEjectMotor] but not upper motor because calls should not be made
   * against upper motor - it is a follower
   */
  private LazyTalonFX[] motorArray = new LazyTalonFX[2];

  @BeforeEach
  public void setup() {
    closeable = MockitoAnnotations.openMocks(intakeSubsystem);
    motorArray[0] = lowerMotor;
    motorArray[1] = rightEjectMotor;
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

  /**
   * Initialize array of target motor percents
   *
   * @param lower lower motor target percent [0]
   * @param upper upper motor target percent [1]
   * @param idler idler motor target percent [2]
   * @return 3 double array of the percents
   */
  private static double[] targetMotorPercents(double lower, double idler) {
    return new double[] {lower, idler};
  }

  private static Stream<Arguments> modeMotorStatesProvider() {
    return Stream.of(
        Arguments.of(Modes.OFF, targetMotorPercents(0, 0)),
        Arguments.of(Modes.IDLING, targetMotorPercents(0, EjectRollers.IDLE)),
        Arguments.of(Modes.INTAKE, targetMotorPercents(IntakeRollers.INTAKE, EjectRollers.IDLE)),
        Arguments.of(Modes.OUTTAKE, targetMotorPercents(IntakeRollers.OUTTAKE, EjectRollers.IDLE)),
        Arguments.of(Modes.EJECT, targetMotorPercents(IntakeRollers.INTAKE, EjectRollers.EJECT)));
  }

  @RobotParamTest
  @MethodSource("modeMotorStatesProvider")
  public void motorStatesMatchConstantsForMode(Modes mode, double[] motorPercentArray) {
    intakeSubsystem.setMode(mode);
    tick();
    for (int i = 0; i < motorArray.length; i++) {
      verify(
              motorArray[i],
              times(1)
                  .description(
                      String.format(
                          "mode %s should have %s percent %s",
                          mode, i == 0 ? "intake" : "idler", motorPercentArray[i])))
          .set(TalonFXControlMode.PercentOutput, motorPercentArray[i]);
    }
  }
}
