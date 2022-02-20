package frc.util;

import static org.junit.jupiter.api.Assertions.fail;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.UtilTest;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mock;
import org.mockito.MockitoAnnotations;
import org.mockito.junit.jupiter.MockitoExtension;

@ExtendWith(MockitoExtension.class)
public class LazyTalonFXTest {
  private AutoCloseable closeable;

  private LazyTalonFX lazyTalonFX;

  @Mock private TalonFX talonFX = new TalonFX(1);

  @BeforeEach
  public void setup() throws Exception {
    closeable = MockitoAnnotations.openMocks(talonFX);
    PowerMockito.whenNew(TalonFX.class).withArguments(1).thenReturn(talonFX);
    lazyTalonFX = new LazyTalonFX(1);
  }

  @AfterEach
  public void teardown() {
    try {
      closeable.close();
    } catch (Exception e) {
      fail(e);
    }
  }

  @UtilTest
  public void test() {
    lazyTalonFX.set(TalonFXControlMode.PercentOutput, .1);
    verify(talonFX, times(1).description(String.format("")))
        .set(TalonFXControlMode.PercentOutput, .1);
  }
}
