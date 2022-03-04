// package frc.robot;

// import static org.junit.jupiter.api.Assertions.assertEquals;
// import static org.junit.jupiter.api.Assertions.assertNotEquals;
// import static org.junit.jupiter.api.Assertions.fail;
// import static org.mockito.Mockito.doCallRealMethod;
// import static org.mockito.Mockito.when;

// import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
// import com.ctre.phoenix.motorcontrol.can.TalonFX;
// import edu.wpi.first.wpilibj.DigitalInput;
// import frc.robot.subsystems.ElevatorSubsystem;
// import org.junit.jupiter.api.AfterEach;
// import org.junit.jupiter.api.BeforeEach;
// import org.junit.jupiter.api.Test;
// import org.junit.jupiter.api.extension.ExtendWith;
// import org.mockito.InjectMocks;
// import org.mockito.Mock;
// import org.mockito.MockitoAnnotations;
// import org.mockito.junit.jupiter.MockitoExtension;

// @ExtendWith(MockitoExtension.class)
// public class ElevatorSubsystemTests {

//   private AutoCloseable closeable;

//   @InjectMocks ElevatorSubsystem elevatorSubsystem;

//   @Mock private TalonFX right;
//   @Mock private TalonFX left;
//   @Mock private DigitalInput bottomLimitSwitch;
//   @Mock private DigitalInput topLimitSwitch;

//   @BeforeEach
//   public void setup() {
//     closeable = MockitoAnnotations.openMocks(elevatorSubsystem);
//   }

//   @AfterEach
//   public void shutdown() {
//     doCallRealMethod().when(topLimitSwitch).close();
//     topLimitSwitch.close();
//     doCallRealMethod().when(bottomLimitSwitch).close();
//     bottomLimitSwitch.close();

//     try {
//       closeable.close();
//     } catch (Exception e) {
//       fail(e);
//     }
//   }

//   private void tick() {
//     elevatorSubsystem.periodic();
//   }

//   /**
//    * call the periodic methods repeatedly
//    *
//    * <p>we call tick to let the intake react to our input - periodic ticks need to occur for our
//    * mocked motors to have updates. The updates will happen for each tick, so using multiple
// ticks
//    * can ensure behavior is consistent
//    */
//   private void tick(int amount) {
//     for (int i = 0; i < amount; i++) elevatorSubsystem.periodic();
//   }

//   @Test
//   public void sanityCheck() {}

//   /** Tests whether X button works. AKA, if we press it, it goesdown to the bottom */
//   /*
//     @Test
//     public void XButtonTest() {

//       elevatorSubsystem.setTargetHeight(0.0);
//       tick();
//       assertEquals(0, elevatorSubsystem.getHeight());
//       // spotless gradle no mad
//     }
//   */
//   @Test
//   public void sensorPositionIsZeroAtStart() {

//     when(right.getSelectedSensorPosition()).thenReturn(0d);

//     assertEquals(0, elevatorSubsystem.getsensorposition());
//   }

//   @Test
//   public void setPower() {

//     right.set(TalonFXControlMode.PercentOutput, 1);
//     assertNotEquals(0, elevatorSubsystem.getsensorposition());
//   }

//   @Test
//   public void YButtonTest() {

//     elevatorSubsystem.setTargetHeight(20.0);
//     tick(10);
//     assertEquals(20.0, elevatorSubsystem.getHeight());
//     // spotless gradle no mad
//   }
// }
