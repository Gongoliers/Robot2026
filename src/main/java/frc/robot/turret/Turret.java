package frc.robot.turret;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.MultithreadedSubsystem;
import frc.robot.azimuth.Azimuth;
import frc.robot.hood.Hood;
import frc.robot.shooter.Shooter;

/** Turret subsystem */
public class Turret extends MultithreadedSubsystem {
  
  /** Turret subsystem singleton */
  private static Turret instance = null;

  /** Azimuth subsystem reference */
  private final Azimuth azimuth;

  /** Hood subsystem reference */
  private final Hood hood;

  /** Shooter subsystem reference */
  private final Shooter shooter;

  /** Current turret state */
  private TurretState state;

  // Variables used by control states

  /** Hub target */
  private Translation2d hubTarget;

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

    state = TurretState.STOW;
  }

  @Override
  public void initializeTab() {
    ShuffleboardTab tab = Shuffleboard.getTab("Turret");

    tab.addString("State", () -> state.name());
  }

  @Override
  public void periodic() {}

  @Override
  public void fastPeriodic() {
    switch (state) {
      case STOW:
        azimuth.setSetpoint(Rotations.of(0.0));
        hood.setSetpoint(hood.getMinPosition());
        shooter.setSetpoint(RotationsPerSecond.of(0.0));
        break;
      case SCORING:
        targetHub();
        break;
    }
  }

  /** Autoaim the turret to shoot in the hub, automagically firing when locked on */
  private void targetHub() {
    // gurt
  }

  /**
   * Stows the turret
   * 
   * @return a command that stows the turret while running
   */
  public Command stow() {
    return Commands.run(() -> {
      state = TurretState.STOW;
    }, this, azimuth, hood, shooter);
  }

  /**
   * Targets a position on the field to try and fire hub shots at
   * 
   * @param targetPosition 2d projection of the positon to aim ait (will aim to fire about 7ft above the given point)
   * @return a command that targets a position on the field to fire hub shots at
   */
  public Command scoreAtTarget(Translation2d targetPosition) {
    return Commands.run(() -> {
      hubTarget = targetPosition;
      state = TurretState.SCORING;
    }, this, azimuth, hood, shooter);
  }
}
