package frc.robot.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.motors.MotorValues;

/** Class with methods to get sysid routine commands */
public class ShooterTester {
  
  /** Shooter tester instance */
  private static ShooterTester instance = null;

  /** Shooter subsystem reference */
  private final Shooter shooter;

  /** Sysid config */
  private final SysIdRoutine.Config config;

  /** Sysid mechanism */
  private final SysIdRoutine.Mechanism mechanism;

  /** Sysid routine */
  private final SysIdRoutine routine;

  /** Voltage out variable used for manual shooter control */
  private final MutVoltage voltageOut;

  /**
   * Gets shooter tester instance
   * 
   * @return shooter tester instance
   */
  public static ShooterTester getInstance() {
    if (instance == null) {
      instance = new ShooterTester();
    }

    return instance;
  }

  private ShooterTester() {
    shooter = Shooter.getInstance();

    voltageOut = Volts.mutable(0.0);

    config = new SysIdRoutine.Config(
      Volts.per(Second).of(0.2),
      Volts.of(7),
      Seconds.of(20));
    
    mechanism = new SysIdRoutine.Mechanism(
      this::setVoltage, 
      this::logMotors, 
      shooter);

    routine = new SysIdRoutine(config, mechanism);
  }

  private void setVoltage(Voltage volts) {
    voltageOut.mut_replace(volts);
  }

  private void logMotors(SysIdRoutineLog log) {
    MotorValues motorValues = shooter.getValues();

    log.motor("drive")
      .voltage(motorValues.motorVoltage)
      .current(motorValues.supplyCurrent)
      .angularPosition(motorValues.position)
      .angularVelocity(motorValues.velocity)
      .angularAcceleration(motorValues.acceleration);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return Commands.race(
      shooter.runAtVoltage(() -> voltageOut),
      routine.quasistatic(direction));
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return Commands.race(
      shooter.runAtVoltage(() -> voltageOut),
      routine.dynamic(direction));
  }
}
