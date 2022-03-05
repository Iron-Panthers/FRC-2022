package frc.util;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.Mockito.when;

import edu.wpi.first.wpilibj.XboxController;
import frc.UtilParamTest;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.extension.ExtendWith;
import org.junit.jupiter.params.provider.ValueSource;
import org.mockito.InjectMocks;
import org.mockito.Mock;
import org.mockito.MockitoAnnotations;
import org.mockito.junit.jupiter.MockitoExtension;

@ExtendWith(MockitoExtension.class)
public class DPadClusterTests {
  @Mock XboxController xboxController;

  @InjectMocks DPadCluster dPadCluster;

  @BeforeEach
  public void setup() {
    MockitoAnnotations.openMocks(dPadCluster);
  }

  @UtilParamTest
  @ValueSource(ints = {315, 0, 45})
  public void pressingUpMakesUpTrue(int angle) {
    when(xboxController.getPOV()).thenReturn(angle);
    assertTrue(
        dPadCluster.getUpButton(),
        String.format("when pov angle is %s dpadcluster should report up as true", angle));
  }

  @UtilParamTest
  @ValueSource(ints = {2 * 45, 3 * 45, 6 * 45})
  public void pressingNonUpMakesUpFalse(int angle) {
    when(xboxController.getPOV()).thenReturn(angle);
    assertFalse(
        dPadCluster.getUpButton(),
        String.format(
            "when pov angle is %s dpadcluster should report up as false, because the angle is more then 45 degrees from cardinal",
            angle));
  }
}
