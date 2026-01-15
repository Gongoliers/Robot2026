package frc.lib;

import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

public class Tunable<R> implements Supplier<Optional<R>> {

    private final DoubleSupplier value;

    private final BooleanSupplier isEnabled;

    private final Function<Double, R> constructor;

    public Tunable(String name, Function<Double, R> constructor, double defaultValue) {
        this.value = createValue(name, defaultValue);
        this.isEnabled = createIsEnabled(name, false);
        this.constructor = constructor;
    }

    public Tunable(String name, Function<Double, R> constructor) {
        this(name, constructor, 0.0);
    }

    public static <U extends Unit> Tunable<Measure<U>> ofUnit(String name, U Unit) {
        String nameWithUnit = String.format("%s (%s)", name, Unit.name());
        return new Tunable<>(nameWithUnit, d -> (Measure<U>) Unit.of(d));
    }

    private static DoubleSupplier createValue(String name, double defaultValue) {
        DoubleTopic topic = NetworkTableInstance.getDefault().getDoubleTopic(name + "/Value");
        // Create a publisher for the topic and set the default value to
        // ensure that the publisher is initially shown in NetworkTables
        topic.publish().set(defaultValue);
        return topic.subscribe(defaultValue);
    }

    private static BooleanSupplier createIsEnabled(String name, boolean defaultValue) {
        BooleanTopic topic = NetworkTableInstance.getDefault().getBooleanTopic(name + "/IsEnabled");
        // Create a publisher for the topic and set the default value to
        // ensure that the publisher is initially shown in NetworkTables
        topic.publish().set(defaultValue);
        return topic.subscribe(defaultValue);
    }

    @Override
    public Optional<R> get() {
        if (isEnabled.getAsBoolean()) {
            return Optional.of(constructor.apply(value.getAsDouble()));
        }
        return Optional.empty();
    }

}
