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
    hoodMapHub.put(2.14, 0.0277);
    hoodMapHub.put(2.61, 0.0277);
    hoodMapHub.put(3.25, 0.0277);
    hoodMapHub.put(3.03, 0.0277);
    hoodMapHub.put(4.0, 0.03377);
    hoodMapHub.put(4.65, 0.03577);
    hoodMapHub.put(5.2, 0.04077);
    hoodMapHub.put(5.7, 0.04777);
    hoodMapHub.put(6.5, 0.04777);

    shooterMapHub.put(1.75, 28.525);
    shooterMapHub.put(2.14, 28.525);
    shooterMapHub.put(2.61, 32.4);
    shooterMapHub.put(3.25, 36.525);
    shooterMapHub.put(3.03, 35.7);
    shooterMapHub.put(4.0, 40.275);
    shooterMapHub.put(4.65, 42.875);
    shooterMapHub.put(5.2, 44.075);
    shooterMapHub.put(5.7, 46.95);
    shooterMapHub.put(6.5, 47.35);
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
