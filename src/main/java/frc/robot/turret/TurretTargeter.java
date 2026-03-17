package frc.robot.turret;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

/** Class with static methods to get hood and shooter setpoints from distances to the target */
public class TurretTargeter {
  
  /** Maps projected distances from the hub in meters to hood setpoints in rotations that shoot into the hub */
  private static final InterpolatingDoubleTreeMap hoodMapHub = new InterpolatingDoubleTreeMap();

  /** Maps projected distances from the hub in meters to shooter setpoints in rotations per second that shoot into the hub */
  private static final InterpolatingDoubleTreeMap shooterMapHub = new InterpolatingDoubleTreeMap();

  static {
    // put tested values in here in format hoodMap.put(distanceMeters, hoodAngleRotations)
  }

  /**
   * Gets a hood angle that will shoot into the hub given the turret's distance from the hub
   * 
   * @param distance the turret's projeced (2d) distance from the hub
   * @return a hood angle that will shoot into the hub
   */
  public static Angle targetHubHood(Distance distance) {
    return Rotations.of(hoodMapHub.get(distance.in(Meters)));
  }

  /**
   * Gets a shooter velocity that will shoot into the hub given the turret's distance from the hub
   * 
   * @param distance the turret's projected (2d) distance from the hub
   * @return a shooter velocity that will shoot into the hub
   */
  public static AngularVelocity targetHubShooter(Distance distance) {
    return RotationsPerSecond.of(shooterMapHub.get(distance.in(Meters)));
  }
}
