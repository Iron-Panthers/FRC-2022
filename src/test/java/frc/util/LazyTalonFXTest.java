package frc.util;

import static org.junit.jupiter.api.Assertions.fail;
import static org.mockito.Mockito.description;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.never;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.verifyNoMoreInteractions;
import static org.mockito.Mockito.when;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.UtilTest;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.InjectMocks;
import org.mockito.Mock;
import org.mockito.MockitoAnnotations;
import org.mockito.junit.jupiter.MockitoExtension;

@ExtendWith(MockitoExtension.class)
public class LazyTalonFXTest {
  private AutoCloseable closeable;

  @Mock private TalonFX talonFX;

  @InjectMocks LazyTalonFX lazyTalonFX = new LazyTalonFX(1);

  @BeforeEach
  public void setup() throws Exception {
    closeable = MockitoAnnotations.openMocks(lazyTalonFX);
  }

  @AfterEach
  public void teardown() {
    try {
      closeable.close();
    } catch (Exception e) {
      fail(e);
    }
  }

  /** make a mocked talon fx that returns the passed id for getBaseID */
  private TalonFX mockMotor(int id) {
    TalonFX mockMotor = mock(TalonFX.class);
    when(mockMotor.getBaseID()).thenReturn(id);
    return mockMotor;
  }

  private void call(int times, Runnable fn) {
    for (int i = 0; i < times; i++) fn.run();
  }

  @UtilTest
  public void setIsCalledButOnlyOnce() {
    for (int i = 0; i < 5; i++) {
      lazyTalonFX.set(TalonFXControlMode.PercentOutput, .1);
    }
    verify(talonFX, times(1).description(String.format("")))
        .set(TalonFXControlMode.PercentOutput, .1);
  }

  @UtilTest
  public void setWithDifferentModeIsCalledForEachMode() {
    TalonFXControlMode lastMode = null;
    for (TalonFXControlMode mode : TalonFXControlMode.values()) {
      lazyTalonFX.set(mode, .3);
      verify(
              talonFX,
              times(1)
                  .description(
                      String.format(
                          "internal talon.set is called with %s after being called with %s",
                          mode, lastMode)))
          .set(mode, .3);
      lastMode = mode;
    }
  }

  @UtilTest
  public void followIsOnlyCalledOnce() {
    int[] ports = {1, 3, 3, 2, 5, 9, 1, 1, 1, 2};
    int lastPort = -1;
    for (int port : ports) {
      TalonFX mockMotor = mockMotor(port);
      lazyTalonFX.follow(mockMotor);
      if (port == lastPort) {
        verify(
                talonFX,
                never()
                    .description(
                        String.format(
                            "internal talon shouldn't have been re-issued the follow command, because the passed motor has id %s == %s",
                            port, lastPort)))
            .follow(mockMotor);
      } else {
        verify(
                talonFX,
                times(1)
                    .description(
                        String.format(
                            "internal talon should be re-issued the follow command, because the passed motor has id %s != %s",
                            port, lastPort)))
            .follow(mockMotor);
      }
      verifyNoMoreInteractions(mockMotor);
      lastPort = port;
    }
  }

  @UtilTest
  public void followIsCalledAgainIfMotorImplicitlyStopsFollowing() {
    TalonFX mockMotor = mockMotor(3);

    call(5, () -> lazyTalonFX.set(TalonFXControlMode.PercentOutput, .5));
    verify(talonFX, description("inner talon fx should be set once"))
        .set(TalonFXControlMode.PercentOutput, .5);

    call(3, () -> lazyTalonFX.follow(mockMotor));
    verify(talonFX, description("inner talon fx should follow once")).follow(mockMotor);

    call(5, () -> lazyTalonFX.set(TalonFXControlMode.PercentOutput, .5));
    verify(
            talonFX,
            times(2)
                .description(
                    "follow should get called again because inner talon implicitly stopped following"))
        .set(TalonFXControlMode.PercentOutput, .5);
  }

  @UtilTest
  public void setIsCalledOncePerNewValue() {
    for (double value = -1; value <= 1; value += .1) {
      for (int i = 0; i < 2; i++) lazyTalonFX.set(TalonFXControlMode.PercentOutput, value);
      verify(
              talonFX,
              description(
                  String.format("inner talon fx .set(PercentOutput, %s) should be called", value)))
          .set(TalonFXControlMode.PercentOutput, value);

      lazyTalonFX.set(TalonFXControlMode.MusicTone, value);
      verify(
              talonFX,
              description(
                  String.format("inner talon fx .set(MusicTone, %s) should be called", value)))
          .set(TalonFXControlMode.MusicTone, value);
    }
  }
}
