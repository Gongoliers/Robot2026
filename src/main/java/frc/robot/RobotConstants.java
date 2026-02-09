package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.EnumSet;
import java.util.Set;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;

/** Global constants for the robot */
public class RobotConstants {

  /** Number of robot periodic calls per second */
  public static final double PERIODIC_RATE = 50;

  /** Duration of each robot periodic call in seconds */
  public static final Time PERIODIC_DURATION = Seconds.of(1.0 / PERIODIC_RATE);

  /** Number of fast peridic calls per second */
  public static final double FAST_PERIODIC_RATE = 100;

  /** Duration of each fast periodic call in seconds */
  public static final Time FAST_PERIODIC_DURATION = Seconds.of(1.0 / FAST_PERIODIC_RATE);

  /** Side lengths of robot chassis */
  public static final Distance CHASSIS_SIDE_LENGTH = Inches.of(24);

  /** Subsystems */
  public enum Subsystem {
    SWERVE,
    SHOOTER,
    AZIMUTH,
    HOOD,
  }

  /** Enabled subsystems */
  public static final Set<Subsystem> ENABLED_SUBSYSTEMS = EnumSet.of(Subsystem.HOOD, Subsystem.SHOOTER);
}
