package frc.robot;

import java.util.Arrays;
import java.util.EnumSet;
import java.util.Set;

/**
 * Manages what hardware is enabled.
 * <p>
 * Convention for "enabled" is that the hardware devices can be created without error.
 * A natural use that arises from this convention is that device creation should be guarded by checking if the hardware is enabled.
 */
public class HardwareManager {

    /**
     * The sets of hardware that are being managed.
     * There may be a one-to-one mapping from subsystem to hardware, or a hardware can encapsulate multiple subsystems.
     */
    public enum Hardware {
        SWERVE,
        SHOOTER,
        AZIMUTH,
        HOOD,
        TURRET,
        CLIMB
    }

    /**
     * The set of hardware that is enabled. By default, all hardware is enabled, unless explicitly disabled.
     */
    private static final Set<Hardware> ENABLED_HARDWARE = EnumSet.allOf(Hardware.class);

    /**
     * Disables all hardware.
     */
    public static void disableAll() {
        ENABLED_HARDWARE.clear();
    }

    /**
     * Disables these hardware.
     *
     * @param hardware The hardware to disable.
     */
    public static void disable(Hardware... hardware) {
        for (Hardware h : hardware) {
            ENABLED_HARDWARE.remove(h);
        }
    }

    /**
     * Enables these hardware.
     *
     * @param hardware The hardware to enable.
     */
    public static void enable(Hardware... hardware) {
        // Equivalent to Collections.addAll(ENABLED_HARDWARE, hardware)
        for (Hardware h : hardware) {
            ENABLED_HARDWARE.add(h);
        }
    }

    /**
     * Guarantees that these hardware sets are enabled.
     *
     * @param hardware The hardware to enable.
     */
    public static void enableOnly(Hardware... hardware) {
        disableAll();
        enable(hardware);
    }

    /**
     * Tests if this hardware is enabled.
     *
     * @param hardware The hardware to test.
     * @return True if this hardware is enabled, false otherwise.
     */
    public static boolean isEnabled(Hardware hardware) {
        return Robot.isReal() && ENABLED_HARDWARE.contains(hardware);
    }

    /**
     * Tests if any of these hardware are enabled.
     *
     * @param hardware The hardware to test.
     * @return True if any of these hardware are enabled, false otherwise.
     */
    public static boolean anyEnabled(Hardware... hardware) {
        return Arrays.stream(hardware).anyMatch(HardwareManager::isEnabled);
    }

}
