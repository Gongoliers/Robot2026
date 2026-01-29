package frc.robot.azimuth;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.commands.runFullSysId;
import frc.lib.motors.MotorValues;

/** Class with methods run sysid with azimuth */
public class AzimuthTester {
  
  /** Azimuth tester instance */
  private static AzimuthTester instance = null;

  /** Azimuth subsystem reference */
  private final Azimuth azimuth;

  /** Sysid config */
  private final SysIdRoutine.Config config;

  /** Sysid mechanism */
  private final SysIdRoutine.Mechanism mechanism;

  /** Sysid routine */
  private final SysIdRoutine routine;

  /** Voltage out variable used for manual azimuth control */
  private final MutVoltage voltageOut;

  /**
   * Gets azimuth tester instance
   * 
   * @return azimuth tester instance
   */
  public static AzimuthTester getInstance() {
    if (instance == null) {
      instance = new AzimuthTester();
    }

    return instance;
  }

  private AzimuthTester() {
    azimuth = Azimuth.getInstance();

    voltageOut = Volts.mutable(0.0);

    config = new SysIdRoutine.Config(
      Volts.per(Second).of(1),
      Volts.of(5),
      Seconds.of(5));

    mechanism = new SysIdRoutine.Mechanism(
      this::setVoltage, 
      this::logMotors, 
      azimuth);

    routine = new SysIdRoutine(config, mechanism);
  }

  private void setVoltage(Voltage volts) {
    voltageOut.mut_replace(volts);
  }

  private void logMotors(SysIdRoutineLog log) {
    MotorValues motorValues = azimuth.getValues();

    log.motor("drive")
      .voltage(motorValues.motorVoltage)
      .current(motorValues.statorCurrent)
      .angularPosition(motorValues.position)
      .angularVelocity(motorValues.velocity)
      .angularAcceleration(motorValues.acceleration);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return Commands.race(
      azimuth.runAtVoltage(() -> voltageOut),
      routine.quasistatic(direction)
    );
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return Commands.race(
      azimuth.runAtVoltage(() -> voltageOut),
      routine.dynamic(direction)
    );
  }

  /**
   * Returns a command that runs quasistatic and dynamic sysid forwards and backwards
   * 
   * @return a command that runs quasistatic and dynamic sysid forwards and backwards
   */
  public Command runFullSysId() {
    return new runFullSysId(
      this::sysIdQuasistatic, 
      this::sysIdDynamic, 
      () -> azimuth.getValues().velocity.abs(RotationsPerSecond) < 0.1,
      azimuth);
  }
}
