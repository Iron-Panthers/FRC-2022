package frc.robot;

import static org.junit.jupiter.api.Assertions.assertNotSame;
import static org.junit.jupiter.api.Assertions.assertSame;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.hal.HAL;
import frc.RobotTest;
import frc.robot.subsystems.IntakeSubsystem;
import frc.util.TalonFXSimFactory;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;

public class IntakeSubsystemTest {
  private TalonFXSimFactory simFactory;

  private IntakeSubsystem intakeSubsystem;
  private TalonFX lowerMotor;
  private TalonFX upperMotor;
  private TalonFX idlerMotor;

  @BeforeEach
  public void setup() {
    // timeout, mode
    // timeout in ms
    assert HAL.initialize(500, 0);
    intakeSubsystem = new IntakeSubsystem();
  }

  @AfterEach
  public void shutdown() {
    intakeSubsystem.close();
  }

  @RobotTest
  public void intakeDefaultsToOff() {
    assertNotSame(IntakeSubsystem.Modes.EJECT, intakeSubsystem.getMode());
    assertSame(IntakeSubsystem.Modes.OFF, intakeSubsystem.getMode());
  }
}
