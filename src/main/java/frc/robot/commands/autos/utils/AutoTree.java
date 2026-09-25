package frc.robot.commands.autos.utils;

import java.util.List;
import java.util.stream.Collectors;

public class AutoTree {
    public enum Event {
        CURRENT,
        WPI,
        DCMP;
        public static final List<AutoTree.Event> ALL = List.of(CURRENT, WPI, DCMP);
    }

    private Event event;

    public AutoTree(Event event) {
        this.event = event;
    }

    private String eventConcat(String auto) {
        return this.event.toString() + '_' + auto;
    }

    public List<String> choicesFrom(List<String> autos) {
        return autos.stream().map(this::eventConcat).collect(Collectors.toList());
    }
}
