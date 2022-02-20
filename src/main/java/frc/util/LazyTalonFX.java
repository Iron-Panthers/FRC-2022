package frc.util;

import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class LazyTalonFX extends TalonFX {

  private static final TalonFXControlMode DEFAULT_MODE = null;
  private static final double DEFAULT_VALUE = 0d;
  private static final int DEFAULT_MASTER_TO_FOLLOW_ID =
      -1; // a can id should never be negative, so no master should ever have this property

  private TalonFXControlMode mode = DEFAULT_MODE;
  private double value = DEFAULT_VALUE;
  private int masterToFollowId = DEFAULT_MASTER_TO_FOLLOW_ID;

  public LazyTalonFX(int deviceNumber) {
    super(deviceNumber);
  }

  private boolean sameSetValues(TalonFXControlMode mode, double value) {
    return this.mode == mode && this.value == value;
  }

  private boolean sameFollowValues(IMotorController masterToFollow) {
    return masterToFollow.getBaseID() == masterToFollowId;
  }

  private void applyValues(TalonFXControlMode mode, double value, IMotorController masterToFollow) {
    this.mode = mode;
    this.value = value;
    this.masterToFollowId = masterToFollow.getBaseID();
  }

  @Override
  public void set(TalonFXControlMode mode, double value) {
    if (!sameSetValues(mode, value)) {
      super.set(mode, value);
      applyValues(mode, value, null);
    }
  }

  @Override
  public void follow(IMotorController masterToFollow) {
    if (!sameFollowValues(masterToFollow)) {
      super.follow(masterToFollow);
      applyValues(null, 0.0, masterToFollow);
    }
  }
}
