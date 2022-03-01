package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.fail;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.robot.subsystems.ElevatorSubsystem;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.InjectMocks;
import org.mockito.Mock;
import org.mockito.MockitoAnnotations;

public class ElevatorSubsystemTests {

  private AutoCloseable closeable;

  @InjectMocks private ElevatorSubsystem elevatorSubsystem;

  @Mock private TalonFX right;
  @Mock private TalonFX left;

  @BeforeEach
  public void setup() {
    right = new TalonFX(Constants.Elevator.Ports.RIGHT_MOTOR);
    left = new TalonFX(Constants.Elevator.Ports.LEFT_MOTOR);
    elevatorSubsystem = new ElevatorSubsystem();
    closeable = MockitoAnnotations.openMocks(elevatorSubsystem);
  }

  @AfterEach
  public void shutdown() {
    elevatorSubsystem.close();
    try {
      closeable.close();
    } catch (Exception e) {
      fail(e);
    }
  }

  private void tick() {
    elevatorSubsystem.periodic();
  }

  public double sensors() {
    return elevatorSubsystem.getsensorposition();
  }

  /**
   * call the periodic methods repeatedly
   *
   * <p>we call tick to let the intake react to our input - periodic ticks need to occur for our
   * mocked motors to have updates. The updates will happen for each tick, so using multiple ticks
   * can ensure behavior is consistent
   */
  private void tick(int amount) {
    for (int i = 0; i < amount; i++) elevatorSubsystem.periodic();
  }

  /** Tests whether X button works. AKA, if we press it, it goesdown to the bottom */
  /*
    @Test
    public void XButtonTest() {

      elevatorSubsystem.setTargetHeight(0.0);
      tick();
      assertEquals(0, elevatorSubsystem.getHeight());
      // spotless gradle no mad
    }
  */
  @Test
  public void getSensor() {

    assertEquals(0, elevatorSubsystem.getsensorposition());
  }

  @Test
  public void setPower() {

    right.set(TalonFXControlMode.PercentOutput, 1);
    assertEquals(0, sensors());
  }

  @Test
  public void YButtonTest() {

    elevatorSubsystem.setTargetHeight(20.0);
    tick(10000);
    assertEquals(20.0, elevatorSubsystem.getHeight());
    // spotless gradle no mad
  }
}
