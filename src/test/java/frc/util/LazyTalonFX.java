package frc.util;

import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class LazyTalonFX extends TalonFX {

  private TalonFXControlMode mode = null;
  private double value = 0.0;
  private IMotorController masterToFollow = null;

  public LazyTalonFX(int deviceNumber) {
    super(deviceNumber);
  }

  private boolean sameSetValues(TalonFXControlMode mode, double value) {
    return this.mode == mode && this.value == value;
  }

  private void applyValues(TalonFXControlMode mode, double value, TalonFX masterToFollow) {
    this.mode = mode;
    this.value = value;
    this.masterToFollow = masterToFollow;
  }

  @Override
  public void set(TalonFXControlMode mode, double value) {
    if (!sameSetValues(mode, value)) {
      super.set(mode, value);
      applyValues(mode, value, null);
    }
  }

  @Override
  public void follow(IMotorController masterToFollow) {}
}
