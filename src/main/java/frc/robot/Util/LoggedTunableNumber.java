package frc.robot.Util;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * Class for a tunable number. Always pulls from NetworkTables,
 * falling back to default if not yet set.
 */
public class LoggedTunableNumber implements DoubleSupplier {
  private static final String tableKey = "/Tuning";

  private final String key;
  private boolean hasDefault = false;
  private double defaultValue;
  private LoggedNetworkNumber dashboardNumber;
  private Map<Integer, Double> lastHasChangedValues = new HashMap<>();

  public LoggedTunableNumber(String dashboardKey) {
    this.key = tableKey + "/" + dashboardKey;
  }

  public LoggedTunableNumber(String dashboardKey, double defaultValue) {
    this(dashboardKey);
    initDefault(defaultValue);
  }

  /**
   * Set the default value of the number. Can only be set once.
   */
  public void initDefault(double defaultValue) {
    if (!hasDefault) {
      hasDefault = true;
      this.defaultValue = defaultValue;

      // ALWAYS initialize NetworkTables entry
      dashboardNumber = new LoggedNetworkNumber(key, defaultValue);
    }
  }

  /**
   * Always return dashboard value (falls back internally to default)
   */
  public double get() {
    if (!hasDefault) {
      return 0.0;
    }
    return dashboardNumber.get();
  }

  public boolean hasChanged(int id) {
    double currentValue = get();
    Double lastValue = lastHasChangedValues.get(id);

    if (lastValue == null || currentValue != lastValue) {
      lastHasChangedValues.put(id, currentValue);
      return true;
    }

    return false;
  }

  public static void ifChanged(
      int id, Consumer<double[]> action, LoggedTunableNumber... tunableNumbers) {

    if (Arrays.stream(tunableNumbers).anyMatch(t -> t.hasChanged(id))) {
      action.accept(
          Arrays.stream(tunableNumbers)
              .mapToDouble(LoggedTunableNumber::get)
              .toArray());
    }
  }

  public static void ifChanged(
      int id, Runnable action, LoggedTunableNumber... tunableNumbers) {

    ifChanged(id, values -> action.run(), tunableNumbers);
  }

  @Override
  public double getAsDouble() {
    return get();
  }
}