package frc.robot.turret;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.lib.InterpolatingMeasureMap;

import static edu.wpi.first.units.Units.*;

/** Class with static methods to get hood and shooter setpoints from distances to the target */
public class TurretTargeter {
  
  /** Maps projected distances from the hub in meters to hood setpoints in rotations that shoot into the hub */
  private static final InterpolatingMeasureMap<DistanceUnit, AngleUnit> hoodMapHub = new InterpolatingMeasureMap<>(Meters, Rotations);

  /** Maps projected distances from the hub in meters to shooter setpoints in rotations per second that shoot into the hub */
  private static final InterpolatingMeasureMap<DistanceUnit, AngularVelocityUnit> shooterMapHub = new InterpolatingMeasureMap<>(Meters, RotationsPerSecond);

  static {
    hoodMapHub.put(1.46, 0.0277);
    hoodMapHub.put(1.98, 0.0277);
    hoodMapHub.put(2.286, 0.0277);
    hoodMapHub.put(2.638, 0.03388);
    hoodMapHub.put(3.045, 0.038333);
    hoodMapHub.put(3.43, 0.043);
    hoodMapHub.put(3.9, 0.04388);
    hoodMapHub.put(4.4, 0.044166);
    hoodMapHub.put(5.07, 0.04944);

    shooterMapHub.put(1.46, 24.25);
    shooterMapHub.put(2.024, 28.0);
    shooterMapHub.put(2.65, 30.0);
    shooterMapHub.put(3.14, 30.25);
    shooterMapHub.put(3.76, 34.0);
    shooterMapHub.put(4.21, 36.75);
    shooterMapHub.put(4.59, 37.25);
    shooterMapHub.put(5.01, 38.75);
    shooterMapHub.put(5.07, 38.25);
  }

  /**
   * Gets a hood angle that will shoot into the hub given the turret's distance from the hub
   * 
   * @param distance the turret's projeced (2d) distance from the hub
   * @return a hood angle that will shoot into the hub
   */
  public static Angle targetHubHood(Distance distance) {
    return (Angle) hoodMapHub.get(distance);
  }

  /**
   * Gets a shooter velocity that will shoot into the hub given the turret's distance from the hub
   * 
   * @param distance the turret's projected (2d) distance from the hub
   * @return a shooter velocity that will shoot into the hub
   */
  public static AngularVelocity targetHubShooter(Distance distance) {
    return (AngularVelocity) shooterMapHub.get(distance);
  }
}
