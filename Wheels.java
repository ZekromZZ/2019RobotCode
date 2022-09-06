package robot.subsystems;


import edu.wpi.first.wpilibj.Spark;

public class Wheels{
    private Spark wheels;

    public Wheels() {
        wheels = new Spark(4);
    }

    public void runWheels(double speed) {
        wheels.set(speed);
    }

    public void stopWheels() {
        runWheels(0);
    }
}