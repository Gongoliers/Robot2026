package frc.robot.intake;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;

public enum IntakePivotState {
  STOW(Rotations.of(0.275)),
  OUT(Rotations.of(-0.01)),
  MOVING(Rotations.of(99999));
  
  private final Angle position;

  IntakePivotState(Angle position) {
    this.position = position;
  }

  public Angle getPosition() {
    return position;
  }
}
