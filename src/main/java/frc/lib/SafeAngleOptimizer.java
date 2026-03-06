package frc.lib;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DriverStation;

/** 
 * Class used to optimize angle setpoints (instead of changing from 170 to -170, change 
 * from 170 to 190), within a certain safe range of setpoints
 * 
 * For example, with a safe angle range of (-540, 540):
 * When at position 170 and given setpoint -170, move to 190
 * When at position 520 and given setpoint -20, move back a rotation to 340
 */
public class SafeAngleOptimizer {

  /** The previous setpoint in rotations */
  private double currentAngle;

  /** Minimum safe setpoint in rotations */
  private double minAngle;

  /** Maximum safe setpoint in rotations */
  private double maxAngle;

  /** Center of the range of safe angles in rotations */
  private double center;

  /**
   * Constructs a SafeAngleOptimizer object with a minimum and maximum safe angle, and an initial angle
   * At the moment, this wraps assuming 360 degree angles are coterminal
   * 
   * @param minAngle Minimum safe angle
   * @param maxAngle Maximum safe angle
   * @param initialAngle Initial angle (defaults to the center of the range of angles)
   */
  public SafeAngleOptimizer(Angle minAngle, Angle maxAngle, Angle initialAngle) {
    center = maxAngle.minus(minAngle).div(2).in(Rotations);
    this.minAngle = minAngle.in(Rotations);
    this.maxAngle = maxAngle.in(Rotations);

    currentAngle = initialAngle.in(Rotations);
  }
  
  /**
   * Constructs a SafeAngleOptimizer object with a minimum and maximum safe angle, and an initial angle
   * At the moment, this wraps assuming 360 degree angles are coterminal
   * 
   * @param minAngle Minimum safe angle
   * @param maxAngle Maximum safe angle
   */
  public SafeAngleOptimizer(Angle minAngle, Angle maxAngle) {
    this(minAngle, maxAngle, maxAngle.minus(minAngle).div(2));
  }

  /**
   * Given a setpoint, returns a new coterminal setpoint that minimizes distance from the 
   * previous setpoint without rotating past safe limits
   * 
   * @param newSetpoint Setpoint to optimize
   * @return Optimized setpoint
   */
  public Angle optimizeSetpoint(Angle newSetpoint) {
    double newSetpointRotations = newSetpoint.in(Rotations);

    double distance = currentAngle - newSetpointRotations; // distance from new setpoint to current angle
    double lowEquivalent = newSetpointRotations + Math.floor(distance);
    double highEquivalent = newSetpointRotations + Math.ceil(distance);

    double lowDistance = Math.abs(currentAngle - lowEquivalent);
    double highDistance = Math.abs(currentAngle - highEquivalent);

    if (highDistance < lowDistance) {  // if closer to higher equivalent than lower equivalent
      if (highDistance > maxAngle) {     // if higher equivalent is past max angle use lower anyways
        currentAngle = lowEquivalent;
      } else {                           // otherwise use the closer high equivalent
        currentAngle = highEquivalent;
      }
    } else {                           // if closer to lower equivalent than higher equivalent
      if (lowDistance < minAngle) {      // if lower equivalent is past min angle use higher anyways
        currentAngle = highEquivalent;
      } else {                           // otherwise use the closer low equivalent
        currentAngle = lowEquivalent;   
      }
    }

    return Rotations.of(currentAngle);
  }

  /**
   * Set new position without optimization
   * 
   * @param newPosition New position (will report a warning and do nothing if newPosition isn't between min and max safe angle)
   */
  public void setAbsolutePosition(Angle newPosition) {
    double newPositionRotations = newPosition.in(Rotations);
    if (newPositionRotations >= minAngle && newPositionRotations <= maxAngle) {
      currentAngle = newPosition.in(Rotations);
    } else {
      DriverStation.reportWarning("Failed to set absolute position, new position out of safe range", true);
    }
  }

  /**
   * Gets the current setpoint's distance from the eedges of the range of safe angles
   * 
   * @return The current setpoint's distance from the edges of the range of safe angles
   * Negative values returned measure how far past the edges of the safe range the current setpoint is
   */
  public Angle getCushion() {
    return Rotations.of(Math.min(maxAngle-currentAngle, currentAngle-minAngle));
  }
}
