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

    String treeMapKeyMapper(String in) {
        return in;
    }

    Map<String, Optional<V>> treeMapValMapper(Map<String, Optional<V>> in) {
        return in;
    }

    public LoggedDashboardDashboardChooser(
            String key, Map<String, Map<String, Optional<V>>> treeMap) {

        this.dashboardChooser = new LoggedDashboardChooser<>(key);

        if (treeMap.isEmpty()) {
            this.treeMap = Map.of("null", Map.of());
        } else {
            this.treeMap = new HashMap<>();

            for (Entry<String, Map<String, Optional<V>>> val : treeMap.entrySet()) {
                Map<String, DashboardChoice> map = new HashMap<>();
                map.put("..", new DashboardChoice(".."));

                for (Entry<String, Optional<V>> inner_val : val.getValue().entrySet()) {
                    map.put(
                            inner_val.getKey(),
                            new DashboardChoice(inner_val.getKey(), inner_val.getValue()));
                }
                this.treeMap.put(val.getKey(), map);
            }

            for (var val : treeMap.entrySet()) {
                dashboardChooser.addOption(val.getKey(), new DashboardChoice(val.getKey()));
            }
        }
        this.treeMode = true;

        this.dashboardChooser.onChange(
                val -> {
                    if (treeMode) {
                        dashboardChooser.clearOptions(this.treeMap.get(val.name));
                        treeMode = false;
                    } else {
                        if (val.name == "..") {
                            dashboardChooser.clear();
                            for (var veal : this.treeMap.entrySet()) {
                                dashboardChooser.addOption(
                                        veal.getKey(), new DashboardChoice(veal.getKey()));
                            }

                            this.treeMode = true;
                            return;
                        } else if (val.value.isPresent()) {

                            V value = val.value.get();

                            if (this.onChange.isPresent()) {
                                this.onChange.get().accept(value);
                            }
                        }
                    }
                });

        this.onChange = Optional.empty();
    }
}
