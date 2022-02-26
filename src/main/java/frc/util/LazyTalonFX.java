package frc.util;

import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class LazyTalonFX {

  private static final TalonFXControlMode DEFAULT_MODE = null;
  private static final double DEFAULT_VALUE = 0d;
  private static final int DEFAULT_MASTER_TO_FOLLOW_ID =
      -1; // a can id should never be negative, so no master should ever have this property

  private TalonFXControlMode mode = DEFAULT_MODE;
  private double value = DEFAULT_VALUE;
  private int masterToFollowId = DEFAULT_MASTER_TO_FOLLOW_ID;

  private TalonFX talonFX;

  public LazyTalonFX(int deviceNumber) {
    talonFX = new TalonFX(deviceNumber);
  }

  protected TalonFX getInternalMotor() {
    return talonFX;
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
    this.masterToFollowId =
        masterToFollow == null ? DEFAULT_MASTER_TO_FOLLOW_ID : masterToFollow.getBaseID();
  }

  public void set(TalonFXControlMode mode, double value) {
    if (!sameSetValues(mode, value)) {
      talonFX.set(mode, value);
      applyValues(mode, value, null);
    }
  }

  public void follow(IMotorController masterToFollow) {
    if (!sameFollowValues(masterToFollow)) {
      talonFX.follow(masterToFollow);
      applyValues(null, 0.0, masterToFollow);
    }
  }

  public void follow(LazyTalonFX masterToFollow) {
    follow(masterToFollow.getInternalMotor());
  }
}
