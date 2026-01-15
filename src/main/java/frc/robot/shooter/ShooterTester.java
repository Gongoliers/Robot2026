package frc.robot.shooter;

import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Time;
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

  // Variables for shooter testing

  /** Shooter motor values */
  private MotorValues motorValues;

  /** List of angular velocities measured during testing */
  private ArrayList<AngularVelocity> velocityAccumulator;

  /** How close velocity should be to target velocity to be considered at that velocity */
  private AngularVelocity velocityTolerance;

  /** Times to reach velocity threshold */
  private ArrayList<Time> returnTimes;

  /** Keeps track of how many frames the shooter has been within velocity tolerance */
  private int framesWithinTolerance;

  /** Keeps track of how many frames since shooter disturbance */
  private int framesSinceDisturbance;

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
      Volts.per(Second).of(1),
      Volts.of(7),
      Seconds.of(10));
    
    mechanism = new SysIdRoutine.Mechanism(
      this::setVoltage, 
      this::logMotors, 
      shooter);

    routine = new SysIdRoutine(config, mechanism);

    motorValues = new MotorValues();
    velocityAccumulator = new ArrayList<AngularVelocity>();

  }

  private void setVoltage(Voltage volts) {
    voltageOut.mut_replace(volts);
  }

  private void logMotors(SysIdRoutineLog log) {
    MotorValues motorValues = shooter.getValues();

    log.motor("drive")
      .voltage(motorValues.motorVoltage)
      .current(motorValues.statorCurrent)
      .angularPosition(motorValues.position)
      .angularVelocity(motorValues.velocity)
      .angularAcceleration(motorValues.acceleration);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return Commands.race(
      shooter.runAtVoltage(() -> voltageOut),
      routine.quasistatic(direction)
    );
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return Commands.race(
      shooter.runAtVoltage(() -> voltageOut),
      routine.dynamic(direction)
    );
  }

  public Command findVelocityVariance(AngularVelocity testVelocity) {
    return Commands.runOnce(() -> {
      velocityAccumulator = new ArrayList<AngularVelocity>();
      shooter.setSetpoint(testVelocity);
    })
    .andThen(Commands.waitSeconds(1)) // Let the motor spin up
    .andThen(Commands.race(
      Commands.waitSeconds(5),
      Commands.run(() -> {
        motorValues = shooter.getValues();
        velocityAccumulator.add(motorValues.velocity);
      })
    ))
    .andThen(Commands.runOnce(() -> {
      double maxVelRotationsPerSec = Double.NEGATIVE_INFINITY;
      double minVelRotationsPerSec = Double.POSITIVE_INFINITY;

      for (AngularVelocity velocity : velocityAccumulator) {
        maxVelRotationsPerSec = Math.max(velocity.in(RotationsPerSecond), maxVelRotationsPerSec);
        minVelRotationsPerSec = Math.min(velocity.in(RotationsPerSecond), minVelRotationsPerSec);
      }
    
      double center = (maxVelRotationsPerSec + minVelRotationsPerSec)/2;
      double maxError = maxVelRotationsPerSec - center;

      velocityTolerance = RotationsPerSecond.of(maxError*1.25);
    }));
  }

  public Command runTests(AngularVelocity testVelocity) {
    return Commands.run(() -> {
      motorValues = shooter.getValues();

      boolean inTolerance = MathUtil.isNear(testVelocity.in(RotationsPerSecond), motorValues.velocity.in(RotationsPerSecond), velocityTolerance.in(RotationsPerSecond));

      if (framesSinceDisturbance > 0) { // If frames since disturbance has started counting, continue
        framesSinceDisturbance += 1;
      }

      if (inTolerance) {
        framesWithinTolerance += 1;
      } else {
        framesWithinTolerance = 0;

        if (framesSinceDisturbance == 0) { // If frames since disturbance hasn't started counting, start counting it
          framesSinceDisturbance += 1;
        }
      }

      // If the shooter has been in tolerance for 0.4s
      if (framesWithinTolerance > 20) {
        if (framesSinceDisturbance > 1) { // If this is a recovery from a long disturbance and not just a long period within tolerance, log this
          double secondsSinceDisturbance = (framesSinceDisturbance - 20) * 0.02;
          returnTimes.add(Seconds.of(secondsSinceDisturbance));
          System.out.println("Returned in "+secondsSinceDisturbance+"s");
        }

        framesSinceDisturbance = 0;
      }
    }).finallyDo(() -> {
      shooter.setSetpoint(RotationsPerSecond.of(0));

      System.out.println("Spun up in "+returnTimes.get(0).in(Seconds)+"s");

      double avg = 0.0;
      for (int i = 1; i < returnTimes.size(); i++) {
        avg += returnTimes.get(i).in(Seconds);
      }
      avg /= returnTimes.size()-1;

      System.out.println("Average return time: "+avg+"s");
    });
  }
}