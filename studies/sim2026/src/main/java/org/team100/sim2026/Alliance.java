package org.team100.sim2026;

import org.team100.sim2026.robots.Defender;
import org.team100.sim2026.robots.ExampleRobot;
import org.team100.sim2026.robots.Robot;
import org.team100.sim2026.robots.Scorer;

public class Alliance {
    final String name;
    final Robot robot1, robot2, robot3;

    public Alliance(String name, Robot r1, Robot r2, Robot r3) {
        this.name = name;
        robot1 = r1;
        robot2 = r2;
        robot3 = r3;
    }

    public static Alliance balanced(String name, AllianceColor color, SimRun sim) {
        return new Alliance(
                name,
                new Scorer(color, "1", 50, 8, sim),
                new ExampleRobot(color, "2", 50, 8, sim),
                new Defender(color, "3", 50, 8, sim));
    }

    public static Alliance uniform(String name, AllianceColor color, SimRun sim) {
        return new Alliance(
                name,
                new ExampleRobot(color, "1", 50, 8, sim),
                new ExampleRobot(color, "2", 50, 8, sim),
                new ExampleRobot(color, "3", 50, 8, sim));
    }

        public static Alliance uniform(String name, int capacity, AllianceColor color, SimRun sim) {
        return new Alliance(
                name,
                new ExampleRobot(color, "1", capacity, 8, sim),
                new ExampleRobot(color, "2", capacity, 8, sim),
                new ExampleRobot(color, "3", capacity, 8, sim));
    }
}
