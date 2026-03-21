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
    hoodMapHub.put(1.46, 0.0277);
    hoodMapHub.put(1.98, 0.0277);
    hoodMapHub.put(2.02, 0.0277);
    hoodMapHub.put(2.286, 0.0277);
    hoodMapHub.put(2.56, 0.0277);
    hoodMapHub.put(3.04, 0.03177);
    hoodMapHub.put(3.56, 0.04377);
    hoodMapHub.put(4.11, 0.04877);
    hoodMapHub.put(4.83, 0.05277);

    shooterMapHub.put(1.46, 24.25);
    shooterMapHub.put(2.02, 28.1);
    shooterMapHub.put(2.56, 32.775);
    shooterMapHub.put(3.04, 34.1);
    shooterMapHub.put(3.56, 34.4);
    shooterMapHub.put(4.11, 36.775);
    shooterMapHub.put(4.83, 39.625);
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
