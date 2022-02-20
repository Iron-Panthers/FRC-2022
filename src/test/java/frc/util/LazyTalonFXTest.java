package frc.util;

import static org.junit.Assert.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.UtilTest;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.runner.RunWith;
import org.mockito.Mock;
import org.mockito.MockitoAnnotations;
import org.powermock.api.mockito.PowerMockito;
import org.powermock.core.classloader.annotations.PrepareForTest;
import org.powermock.modules.junit4.PowerMockRunner;

@PrepareForTest(TalonFX.class)
@RunWith(PowerMockRunner.class)
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
    assertTrue(true);
  }
}
