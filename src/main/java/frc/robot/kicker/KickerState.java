package frc.robot.kicker;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;

public enum KickerState {
  STOP(RotationsPerSecond.of(0)),
  TEST(RotationsPerSecond.of(5));

  private final AngularVelocity velocity;

  KickerState(AngularVelocity velocity) {
    this.velocity = velocity;
  }

  public AngularVelocity getVelocity() {
    return velocity;
  }
}
