package frc.util;

import java.util.function.BooleanSupplier;

public class ControllerUtil {
  private ControllerUtil() {}
  /**
   * Compares a layer button with a layer state, and if they match returns the layered button
   *
   * @param layer the boolean supplier that is the layer switch
   * @param layerState the state of the layer switch that is valid
   * @param button the button inside the layer
   * @return true if the layer is enabled and the button is pressed
   */
  public static BooleanSupplier cumBooleanSupplier(
      BooleanSupplier layer, boolean layerState, BooleanSupplier button) {
    return () -> layer.getAsBoolean() == layerState && button.getAsBoolean();
  }
}
