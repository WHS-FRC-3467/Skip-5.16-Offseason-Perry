// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// https://github.com/3015RangerRobotics/2024Public/blob/main/RobotCode2024/src/main/java/frc/robot/util/LoggedDashboardChooser.java

package frc.lib.util;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.littletonrobotics.junction.networktables.LoggedNetworkInput;
import org.wpilib.tunable.Selectable;
import org.wpilib.tunable.Tunables;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;

/**
 * A dashboard chooser that integrates with AdvantageKit's logging system.
 *
 * <p>This class wraps WPILib's {@link Selectable} (the 2027 replacement for SendableChooser) to
 * provide automatic logging of selected values. This version works correctly with AdvantageKit's
 * replay functionality, allowing you to replay autonomous selections and other dashboard choices
 * from log files. It also supports replacing the option set at runtime via {@link #clearOptions}.
 *
 * <p>Example usage:
 *
 * <pre>{@code
 * LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Mode");
 * autoChooser.addDefaultOption("Do Nothing", Commands.none());
 * autoChooser.addOption("Simple Auto", simpleAuto());
 * autoChooser.addOption("Complex Auto", complexAuto());
 *
 * // Get selected command
 * Command selectedAuto = autoChooser.get();
 * }</pre>
 */
public class LoggedDashboardChooser<V> extends LoggedNetworkInput {
    private static final String NO_VALUE_SELECTED = "No Value Selected";

    private final String key;
    private String selectedValue = null;
    private String lastSelected = null;
    private final Selectable<String> selectable = new Selectable<>();
    private Map<String, V> options = new HashMap<>();
    private Consumer<V> listener = null;

    private final LoggableInputs inputs =
            new LoggableInputs() {
                @Override
                public void toLog(LogTable table) {
                    table.put(key, selectedValue);
                }

                @Override
                public void fromLog(LogTable table) {
                    selectedValue = table.get(key, selectedValue);
                }
            };

    /**
     * Creates a new LoggedDashboardChooser.
     *
     * @param key The Tunables key, published under the Tunables table for NT or
     *     "/DashboardInputs/{key}" when logged
     */
    public LoggedDashboardChooser(String key) {
        this.key = key;
        Tunables.publish(key, selectable);
        periodic();
        Logger.registerDashboardInput(this);
    }

    /**
     * Adds a new option to the chooser.
     *
     * @param key Display name for the option
     * @param value Value returned when this option is selected
     */
    public void addOption(String key, V value) {
        selectable.add(key, key);
        options.put(key, value);
    }

    /**
     * Replaces all options with a new set and resets the default to a placeholder with no value.
     *
     * @param newOptions Display name to value mapping for the new options
     */
    public void clearOptions(Map<String, V> newOptions) {
        this.clear();

        for (var option : newOptions.entrySet()) {
            selectable.add(option.getKey(), option.getKey());
            options.put(option.getKey(), option.getValue());
        }
        // Selectable falls back to the default when the dashboard's current selection is no
        // longer an option, so this placeholder (which maps to no value) acts as "deselected".
        selectable.addDefault(NO_VALUE_SELECTED, "  ");
    }

    public void clearSelected() {
        selectedValue = "";
        lastSelected = "";
    }

    public void clear() {
        selectable.clear();
        options.clear();
    }

    /**
     * Adds a new option and sets it as the default.
     *
     * @param key Display name for the default option
     * @param value Value returned when this option is selected
     */
    public void addDefaultOption(String key, V value) {
        selectable.addDefault(key, key);
        options.put(key, value);
    }

    /**
     * Returns the currently selected option value. If no option is selected, returns the default.
     * If no default exists, returns null.
     *
     * @return The selected value, or null if nothing is selected
     */
    public V get() {
        return options.get(selectedValue);
    }

    /**
     * Returns the internal Selectable for dashboard layout configuration. Do not read data from
     * this directly - use {@link #get()} instead.
     *
     * @return The internal Selectable object
     */
    public Selectable<String> getSelectable() {
        return selectable;
    }

    @Override
    public void periodic() {
        if (!Logger.hasReplaySource()) {
            selectedValue = selectable.getSelected();
        }
        Logger.processInputs(prefix, inputs);
        if (selectedValue != null) {

            if (listener != null && !selectedValue.equals(lastSelected)) {
                listener.accept(get());
            }
            lastSelected = selectedValue;
        }
    }

    /**
     * Registers a listener to be called when the selected option changes.
     *
     * @param listener Consumer to be called with the new value when selection changes
     */
    public void onChange(Consumer<V> listener) {
        this.listener = listener;
    }
}
