package frc.robot.spindexer;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;

public enum SpindexerState {
  STOP(RotationsPerSecond.of(0)),
  TEST(RotationsPerSecond.of(5)),
  MOVING(RotationsPerSecond.of(9999));

  private final AngularVelocity velocity;

  SpindexerState(AngularVelocity velocity) {
    this.velocity = velocity;
  }

  public AngularVelocity getVelocity() {
    return velocity;
  }
}
