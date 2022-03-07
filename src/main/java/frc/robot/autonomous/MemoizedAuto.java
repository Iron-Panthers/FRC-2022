package frc.robot.autonomous;

/**
 * an interface for an auto to implement a method for the memoization of expensive computations at
 * robot construction instead of enable
 */
public interface MemoizedAuto {

  public void memoize();
}
