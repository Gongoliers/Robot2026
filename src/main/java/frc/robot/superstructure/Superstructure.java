package frc.robot.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.Subsystem;
import frc.robot.intake.Intake;
import frc.robot.intake.IntakeState;
import frc.robot.kicker.Kicker;
import frc.robot.kicker.KickerState;
import frc.robot.spindexer.Spindexer;
import frc.robot.spindexer.SpindexerState;
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
    if (state == SuperstructureState.SCORE) {
      if (turret.atTargetState()) {
        kicker.setState(KickerState.RUN);
        spindexer.setState(SpindexerState.RUN);
      } else{
        kicker.setState(KickerState.STOP);
        spindexer.setState(SpindexerState.STOP);
      }
    }
  }

  public Command init() {
    return Commands.parallel(
      turret.stow(),
      kicker.goToState(KickerState.STOP),
      spindexer.goToState(SpindexerState.STOP)
    ).andThen(
      intake.goToState(IntakeState.INIT)
    ).finallyDo(() -> state = SuperstructureState.INIT);
  }

  public Command stow() {
    return Commands.parallel(
      turret.stow(),
      kicker.goToState(KickerState.STOP),
      spindexer.goToState(SpindexerState.STOP)
    ).andThen(
      intake.goToState(IntakeState.STOW)
    ).finallyDo(() -> state = SuperstructureState.STOW);
  }

  public Command idle() {
    return Commands.parallel(
      intake.goToState(IntakeState.OUT),
      kicker.goToState(KickerState.STOP),
      spindexer.goToState(SpindexerState.STOP)
    ).andThen(
      turret.stow()
    ).finallyDo(() -> state = SuperstructureState.IDLE);
  }

  public Command faceHub() {
    return Commands.parallel(
      intake.goToState(IntakeState.OUT),
      kicker.goToState(KickerState.STOP),
      spindexer.goToState(SpindexerState.STOP)
    ).andThen(
      turret.faceHub()
    ).finallyDo(() -> state = SuperstructureState.FACE_HUB);
  }

  public Command intake() {
    return Commands.parallel(
      intake.goToState(IntakeState.OUT),
      kicker.goToState(KickerState.STOP),
      spindexer.goToState(SpindexerState.STOP)
    ).andThen(Commands.parallel(
      intake.goToState(IntakeState.INTAKE),
      turret.faceHub()
    )).finallyDo(() -> state = SuperstructureState.INTAKE);
  }

  public Command score() {
    return Commands.parallel(
      intake.goToState(IntakeState.OUT),
      kicker.goToState(KickerState.STOP),
      spindexer.goToState(SpindexerState.STOP)
    ).andThen(Commands.parallel(
      intake.goToState(IntakeState.AGITATE),
      turret.targetHub()
    ))
    .finallyDo(() -> state = SuperstructureState.SCORE);
  }
}
