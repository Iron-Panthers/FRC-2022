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

  public Trigger on(BooleanSupplier boolSupplier) {
    return layerSwitch.and(new Button(boolSupplier));
  }

  public Trigger on(Trigger button) {
    return layerSwitch.and(button);
  }
}
