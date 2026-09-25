package frc.lib.util;

import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Optional;
import java.util.function.Consumer;

public class LoggedDashboardDashboardChooser<V> {

    private class DashboardChoice {
        public Optional<V> value;
        public String name;

        public DashboardChoice(String name, Optional<V> value) {
            this.name = name;
            this.value = value;
        }

        public DashboardChoice(String name) {
            this.name = name;
            this.value = Optional.empty();
        }
    }

    private LoggedDashboardChooser<DashboardChoice> dashboardChooser;
    private boolean treeMode;

    private final Map<String, Map<String, DashboardChoice>> treeMap;

    private Optional<Consumer<V>> onChange;

    public void onChange(Consumer<V> consumer) {
        this.onChange = Optional.of(consumer);
    }

    public Optional<V> get() {
        if (treeMode) {
            return Optional.empty();
        }
        return dashboardChooser.get().value;
    }

    public LoggedDashboardDashboardChooser(
            String key, Map<String, Map<String, Optional<V>>> treeMap) {

        this.dashboardChooser = new LoggedDashboardChooser<>(key);

        if (treeMap.isEmpty()) {
            this.treeMap = Map.of("null", Map.of());
        } else {
            this.treeMap = new HashMap<>();

            for (Entry<String, Map<String, Optional<V>>> entry : treeMap.entrySet()) {
                Map<String, DashboardChoice> map = new HashMap<>();
                map.put("..", new DashboardChoice(".."));

                for (Entry<String, Optional<V>> innerEntry : entry.getValue().entrySet()) {
                    map.put(
                            innerEntry.getKey(),
                            new DashboardChoice(innerEntry.getKey(), innerEntry.getValue()));
                }
                this.treeMap.put(entry.getKey(), map);
            }
            for (var entry : treeMap.entrySet()) {
                dashboardChooser.addOption(entry.getKey(), new DashboardChoice(entry.getKey()));
            }
        }
        this.treeMode = true;

        this.dashboardChooser.onChange(
                choice -> {
                    if (choice == null) {
                        return;
                    }
                    if (treeMode) {

                        dashboardChooser.clearOptions(this.treeMap.get(choice.name));

                        dashboardChooser.clearSelected();

                        treeMode = false;

                    } else {
                        if (choice.name == "..") {
                            dashboardChooser.clear();
                            for (var entry : this.treeMap.entrySet()) {
                                dashboardChooser.addOption(
                                        entry.getKey(), new DashboardChoice(entry.getKey()));
                            }

                            this.treeMode = true;
                            return;
                        } else if (choice.value.isPresent()) {

                            V value = choice.value.get();

                            if (this.onChange.isPresent()) {
                                this.onChange.get().accept(value);
                            }
                        }
                    }
                });

        this.onChange = Optional.empty();
    }
}
