package frc.robot.hood;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.commands.runFullSysId;
import frc.lib.motors.MotorValues;

/** Class with methods to run sysid with hood */
public class HoodTester {
  
  /** Hood tester instance */
  private static HoodTester instance = null;

  /** Hood subsystem reference */
  private final Hood hood;

  /** Sysid config */
  private final SysIdRoutine.Config config;

  /** Sysid mechanism */
  private final SysIdRoutine.Mechanism mechanism;

  /** Sysid routine */
  private final SysIdRoutine routine;

  /** Voltage out variable used for manual hood control */
  private final MutVoltage voltageOut;

  /**
   * Gets hood tester instance
   * 
   * @return hood tester instance
   */
  public static HoodTester getInstance() {
    if (instance == null) {
      instance = new HoodTester();
    }

    return instance;
  }

  private HoodTester() {
    hood = Hood.getInstance();

    voltageOut = Volts.mutable(0.0);

    config = new SysIdRoutine.Config(
      Volts.per(Second).of(0.75),
      Volts.of(2),
      Seconds.of(2));

    mechanism = new SysIdRoutine.Mechanism(
      this::setVoltage,
      this::logMotors, 
      hood);

    routine = new SysIdRoutine(config, mechanism);
  }

  private void setVoltage(Voltage volts) {
    voltageOut.mut_replace(volts);
  }

  private void logMotors(SysIdRoutineLog log) {
    MotorValues motorValues = hood.getValues();

    log.motor("drive")
      .voltage(motorValues.motorVoltage)
      .current(motorValues.statorCurrent)
      .angularPosition(motorValues.position)
      .angularVelocity(motorValues.velocity)
      .angularAcceleration(motorValues.acceleration);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return Commands.race(
      hood.runAtVoltage(() -> voltageOut),
      routine.quasistatic(direction)
    );
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return Commands.race(
      hood.runAtVoltage(() -> voltageOut),
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
      () -> true,
      Seconds.of(0.25),
      hood);
  }
}
