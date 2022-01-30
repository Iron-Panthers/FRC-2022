package frc.util;

import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.BooleanSupplier;

public class Layer {

  private Trigger layerSwitch;

  public Layer(BooleanSupplier layerSwitch) {
    this.layerSwitch = new Button(layerSwitch);
  }

  public Layer(Trigger layerSwitch) {
    this.layerSwitch = layerSwitch;
  }

  public Button on(Trigger button) {
    return new Button(layerSwitch.and(button));
  }

  public Button off(Trigger button) {
    return new Button(layerSwitch.negate().and(button));
  }

  public Button on(BooleanSupplier boolSupplier) {
    return on(new Button(boolSupplier));
  }

  public Button off(BooleanSupplier boolSupplier) {
    return off(new Button(boolSupplier));
  }
}
