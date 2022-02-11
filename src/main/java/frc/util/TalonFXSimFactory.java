package frc.util;

public class TalonFXSimFactory {

  public TalonFXSim get(int port) {
    return new TalonFXSim(port);
  }
}
