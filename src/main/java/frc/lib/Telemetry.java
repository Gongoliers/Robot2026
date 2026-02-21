package frc.lib;

import java.util.HashSet;
import java.util.Set;

/** Handles telemetry with Shuffleboard */
public class Telemetry {

  /** Set of subsytems to be utilized by initializeTabs */
  private static Set<Subsystem> subsystemTabs = new HashSet<Subsystem>();

  /** 
   * registers a Subsytem to have their initializeTab function called
   * 
   * @param subsystem Subsystem class to register
   */
  public static void registerSubsystem(Subsystem subsystem){
    subsystemTabs.add(subsystem);
  }

  /** runs the initializeTabe function for each of the subsytems that have been registered */
  public static void initializeTabs() {
    for (Subsystem subsystem : subsystemTabs) {
      subsystem.initializeTab();
    }
  }
}
