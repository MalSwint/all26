package frc.robot;

import sim.engine.SimState;
import sim.field.continuous.Continuous2D;
import sim.util.Double2D;

public class Students extends SimState {
    public Continuous2D field = new Continuous2D(1.0, 8, 16);
    public int numStudents = 504;
    double forceToSchoolMultiplier = 0.01;
    double randomMultiplier = 0.1;

    public Students(long seed) {
        super(seed);
    }

    public void start() {
        super.start();
        // clear the yard
        field.clear();

        // Student red1 = new Student();
        // Student red2 = new Student();
        // Student red3 = new Student();
        // Student blue1 = new Student();
        // Student blue2 = new Student();
        // Student blue3 = new Student();
        // add some students to the yard

        for (int i = 0; i < numStudents; i++) {
            Student student = new Student();
            field.setObjectLocation(student,
                    new Double2D(field.getWidth() * 0.5 + random.nextDouble() - 0.5,
                            field.getHeight() * 0.5 + random.nextDouble() - 0.5));
            schedule.scheduleRepeating(student);
        }
    }

    public static void main(String[] args) {
        doLoop(Students.class, args);
        System.exit(0);
    }
}