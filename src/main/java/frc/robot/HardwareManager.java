package frc.robot;

import java.util.EnumSet;
import java.util.Set;

public class HardwareManager {

    public enum Hardware {
        SWERVE,
        SHOOTER,
        AZIMUTH,
        HOOD,
        CLIMB
    }

    public static final Set<Hardware> ENABLED_HARDWARE = EnumSet.allOf(Hardware.class);

    public static void disableAll() {
        ENABLED_HARDWARE.clear();
    }

    public static void disable(Hardware... hardware) {
        for (Hardware h : hardware) {
            ENABLED_HARDWARE.remove(h);
        }
    }

    public static void enable(Hardware... hardware) {
        for (Hardware h : hardware) {
            ENABLED_HARDWARE.add(h);
        }
    }

    public static void enableOnly(Hardware... hardware) {
        disableAll();
        enable(hardware);
    }

    public static boolean isEnabled(Hardware hardware) {
        return Robot.isReal() && ENABLED_HARDWARE.contains(hardware);
    }

}
