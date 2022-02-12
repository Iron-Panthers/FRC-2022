package frc.robot;

import static org.junit.jupiter.api.Assertions.assertNotSame;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.fail;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.RobotParamTest;
import frc.RobotTest;
import frc.robot.Constants.Intake;
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

  @Mock private TalonFX lowerMotor;
  @Mock private TalonFX upperMotor;
  @Mock private TalonFX idlerMotor;

  @BeforeEach
  public void setup() {
    closeable = MockitoAnnotations.openMocks(intakeSubsystem);
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
    while ((amount -= 1) >= 0) intakeSubsystem.periodic();
  }

  @AfterEach
  public void shutdown() {
    intakeSubsystem.close();
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
  public void intakeMotorsAndIdlerSpunDuringIntake() {
    intakeSubsystem.setMode(Modes.INTAKE);
    tick();
    verify(lowerMotor).set(TalonFXControlMode.PercentOutput, Intake.INTAKE_PERCENT);
    // there is no need to test upper motor, it is already following lower motor
    verify(idlerMotor).set(TalonFXControlMode.PercentOutput, Intake.IDLER_PERCENT);
  }

  @RobotTest
  public void intakeKeepsSpinning() {
    intakeSubsystem.setMode(Modes.INTAKE);
    int calls = 3;
    tick(calls);
    verify(lowerMotor, times(calls)).set(TalonFXControlMode.PercentOutput, Intake.INTAKE_PERCENT);
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
    tick();
    intakeSubsystem.nextMode();
    assertSame(targetMode, intakeSubsystem.getMode());
    tick();
    assertSame(targetMode, intakeSubsystem.getMode());
  }
}
