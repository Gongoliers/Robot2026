package frc.lib;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.commands.runFullSysId;
import frc.lib.motors.MotorValues;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

/**
 * Utility class for running SysId test routines on mechanisms.
 */
public class SysIdTester {

    /**
     * Subsystem reference for binding commands.
     */
    private final Subsystem subsystem;

    /**
     * Supplier for the subsystems' motor values.
     */
    private final Supplier<MotorValues> motorValuesSupplier;

    /**
     * The SysId routine.
     */
    private final SysIdRoutine routine;

    /**
     * Creates a tester instance for the subsystem.
     *
     * @param subsystem The subsystem to test.
     * @param voltageConsumer A function that accepts a voltage.
     * @param motorValuesSupplier A function that returns motor values.
     * @param config The configuration for the SysId routine.
     */
    public SysIdTester(Subsystem subsystem, Consumer<Voltage> voltageConsumer, Supplier<MotorValues> motorValuesSupplier, SysIdRoutine.Config config) {
        this.subsystem = subsystem;

        SysIdRoutine.Mechanism mechanism = new SysIdRoutine.Mechanism(
                voltageConsumer,
                this::logMotorValues,
                this.subsystem);

        this.motorValuesSupplier = motorValuesSupplier;

        this.routine = new SysIdRoutine(config, mechanism);
    }

    /**
     * Records the subsystems' motor values to the routine's log.
     *
     * @param log The log to record the motor values to.
     */
    private void logMotorValues(SysIdRoutineLog log) {
        MotorValues motorValues = motorValuesSupplier.get();

        log.motor(subsystem.getName())
                .voltage(motorValues.motorVoltage)
                .current(motorValues.statorCurrent)
                .angularPosition(motorValues.position)
                .angularVelocity(motorValues.velocity)
                .angularAcceleration(motorValues.acceleration);
    }

    /**
     * Creates a command that runs a SysId quasistatic routine.
     *
     * @param direction The direction.
     * @return A command that runs the SysId quasistatic routine.
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction);
    }

    /**
     * Creates a command that runs a SysId dynamic routine.
     *
     * @param direction The direction.
     * @return A command that runs the SysId dynamic routine.
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction);
    }

    /**
     * Creates a command that performs a full SysId test.
     * Includes quasistatic and dynamic routines, in both directions.
     *
     * @param isSettledSupplier A function that returns true when the next routine should be run.
     * @param rest The amount of time to rest between routines.
     * @return A command that performs a full SysId test.
     */
    public Command fullSysId(BooleanSupplier isSettledSupplier, Time rest) {
        return new runFullSysId(
                this::sysIdQuasistatic,
                this::sysIdDynamic,
                isSettledSupplier,
                rest,
                this.subsystem
        );
    }

    /**
     * Creates a command that performs a full SysId test without waiting for settling.
     * Includes quasistatic and dynamic routines, in both directions.
     *
     * @param rest The amount of time to rest between routines.
     * @return A command that performs a full SysId test.
     */
    public Command fullSysId(Time rest) {
        return fullSysId(() -> true, rest);
    }

}
