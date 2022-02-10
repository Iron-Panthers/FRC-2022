package frc.robot;

import static org.junit.jupiter.api.Assertions.assertNotSame;
import static org.junit.jupiter.api.Assertions.assertSame;

import frc.RobotTest;
import frc.robot.subsystems.IntakeSubsystem;
import org.junit.jupiter.api.BeforeEach;

public class IntakeSubsystemTest {

  private IntakeSubsystem intakeSubsystem;

  @BeforeEach
  public void reset() {
    intakeSubsystem = new IntakeSubsystem();
  }

  @RobotTest
  public void intakeDefaultsToOff() {
    assertNotSame(IntakeSubsystem.Modes.EJECT, intakeSubsystem.getMode());
    assertSame(IntakeSubsystem.Modes.OFF, intakeSubsystem.getMode());
  }
}
