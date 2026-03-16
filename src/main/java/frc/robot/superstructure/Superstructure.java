package frc.robot.superstructure;

import frc.lib.Subsystem;
import frc.robot.intake.Intake;
import frc.robot.kicker.Kicker;
import frc.robot.spindexer.Spindexer;
import frc.robot.turret.Turret;

/**
 * Superstructure subsystem
 * 
 * Manages automated control of all other subsystems
 */
public class Superstructure extends Subsystem {
  
  /** Superstructure subsystem singleton */
  private static Superstructure instance = null;

  /** Intake subsystem reference */
  private final Intake intake;

  /** Kicker subsystem reference */
  private final Kicker kicker;

  /** Spindexer subsystem reference */
  private final Spindexer spindexer;

  /** Turret subsystem reference */
  private final Turret turret;

  /** Superstructure state */
  private SuperstructureState state;

  /**
   * Gets superstructure subsystem instance
   * 
   * @return superstructure subsystem instance
   */
  public static Superstructure getInstance() {
    if (instance == null) {
      instance = new Superstructure();
    }

    return instance;
  }

  /** Superstructure subsystem constructor */
  private Superstructure() {
    intake = Intake.getInstance();
    kicker = Kicker.getInstance();
    spindexer = Spindexer.getInstance();
    turret = Turret.getInstance();

    state = SuperstructureState.INIT;
  }

  @Override
  public void initializeTab() {

  }

  @Override
  public void periodic() {
    
  }
}
