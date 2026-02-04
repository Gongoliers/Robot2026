package frc.robot.turret;

import frc.robot.azimuth.Azimuth;
import frc.robot.hood.Hood;
import frc.robot.shooter.Shooter;

/** Turret subsystem */
public class Turret {
  
  /** Turret subsystem singleton */
  private static Turret instance = null;

  /** Azimuth subsystem reference */
  private final Azimuth azimuth;

  /** Hood subsystem reference */
  private final Hood hood;

  /** Shooter subsystem reference */
  private final Shooter shooter;

  /**
   * Gets turret subsystem instance
   * 
   * @return turret subsystem instance
   */
  public static Turret getInstance() {
    if (instance == null) {
      instance = new Turret();
    }

    return instance;
  }
  
  /** Turret subsystem constructor */
  private Turret() {
    azimuth = Azimuth.getInstance();
    hood = Hood.getInstance();
    shooter = Shooter.getInstance();
  }
}
