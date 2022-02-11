package frc.util;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class TalonFXSim extends TalonFX {
  private final int port;

  TalonFXSim(int port) {
    super(0);
    this.port = port;
  }

  @Override
  public void set(TalonFXControlMode mode, double value) {}
}
