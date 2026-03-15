package frc.robot.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;

public enum IntakeState {
  START(Rotations.of(0.275), RotationsPerSecond.of(0)),
  STOW(Rotations.of(0.25), RotationsPerSecond.of(0)),
  OUT(Rotations.of(0.01), RotationsPerSecond.of(0)),
  INTAKE(Rotations.of(0.01), RotationsPerSecond.of(50)),
  AGITATE(null, RotationsPerSecond.of(50));

  public final Angle pivotSetpoint;
  public final AngularVelocity rollerSetpoint;

  IntakeState(Angle pivotSetpoint, AngularVelocity rollerSetpoint) {
    this.pivotSetpoint = pivotSetpoint;
    this.rollerSetpoint = rollerSetpoint;
  }
}
