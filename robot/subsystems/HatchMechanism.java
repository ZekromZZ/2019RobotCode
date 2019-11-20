package robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class HatchMechanism {

    private DoubleSolenoid hatchSolenoid;

    public HatchMechanism() {
        hatchSolenoid = new DoubleSolenoid(6, 1, 0);
    }

    public void place() {
        hatchSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void retract() {
        hatchSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

}
