package frc.lib;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;

/**
 * Provides a unit-safe interface for an interpolating double map.
 *
 * @param <KeyUnit> The unit of the map's keys.
 * @param <ValueUnit> The unit of the map's values.
 */
public class InterpolatingMeasureMap<KeyUnit extends Unit, ValueUnit extends Unit> {

    /**
     * Stores a copy of the key's unit, for safe conversion.
     */
    private final KeyUnit KeyUnit;

    /**
     * Stores a copy of the value's unit, for safe conversion.
     */
    private final ValueUnit ValueUnit;

    /**
     * Map of values.
     * Performs the interpolation, after performing unit-safe conversion.
     */
   private final InterpolatingDoubleTreeMap map;

    /**
     * Creates an interpolating measure map.
     *
     * @param KeyUnit The unit of the map's keys.
     * @param ValueUnit The unit of the map's values.
     */
   public InterpolatingMeasureMap(KeyUnit KeyUnit, ValueUnit ValueUnit) {
       map = new InterpolatingDoubleTreeMap();
       this.KeyUnit = KeyUnit;
       this.ValueUnit = ValueUnit;
   }

    /**
     * Directly inserts a key-value pair.
     * The parameters must be in this map's units.
     *
     * @param keyInUnit The key, in this map's key unit.
     * @param valueInUnit The value, in this map's value unit.
     */
   public void put(double keyInUnit, double valueInUnit) {
        map.put(keyInUnit, valueInUnit);
   }

    /**
     * Inserts a key-value pair.
     * The parameters can be in any unit which can be converted to this map's unit.
     *
     * @param key The key.
     * @param value The value.
     */
   public void put(Measure<KeyUnit> key, Measure<ValueUnit> value) {
       put(key.in(KeyUnit), value.in(ValueUnit));
   }

    /**
     * Returns the value associated with a given key.
     * Interpolates if the key is not in the map.
     *
     * @param key The key.
     * @return The value associated with the given key.
     */
   public Measure<ValueUnit> get(Measure<KeyUnit> key) {
       double value = map.get(key.in(KeyUnit));
       return (Measure<ValueUnit>) ValueUnit.of(value);
   }

}
