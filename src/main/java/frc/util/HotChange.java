package frc.util;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

public class HotChange {
  private HotChange() {}

  private static final ShuffleboardTab tab = Shuffleboard.getTab("HotChange");

  private static class Changer implements Sendable {
    String name;
    DoubleSupplier constGetter;
    DoubleConsumer constSetter;

    Changer(String name, DoubleSupplier constGetter, DoubleConsumer constSetter) {
      this.name = name;
      this.constGetter = constGetter;
      this.constSetter = constSetter;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
      builder.addDoubleProperty(name, constGetter, constSetter);
    }
  }

  public static void constant(String name, DoubleSupplier constGetter, DoubleConsumer constSetter) {
    System.out.println("HotChange.constant( name:" + name + " )");
    tab.add(new Changer(name, constGetter, constSetter));
  }
}
