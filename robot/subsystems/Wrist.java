package robot.subsystems;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Spark;

public class Wrist {

    private final int potRange = 2000;//Range of pot in degrees
    private final int potOffset = -1000;//Offset of pot from level

    private final double p = 0.004; //0.008
    private final double i = 0.0002;
    private final double d = 0.0;

    private final double maxUpPIDSpeed = 0.5;
    private final double maxDownPIDSpeed = -0.5;

    private Spark wrist;

    private AnalogPotentiometer wristPot;
    
    private PIDController wristPID;

    public Wrist() {
        wrist = new Spark(5);
        wrist.setInverted(true);
      
        wristPot = new AnalogPotentiometer(1, potRange, potOffset);

        wristPID = new PIDController(p, i, d, wristPot, wrist);
        wristPID.setInputRange(potOffset, potRange+potOffset);
        wristPID.setOutputRange(maxDownPIDSpeed, maxUpPIDSpeed);
    }

    public double pidOutput() {
        return wristPID.get();
    }

    public void setPosition(double pos) {
        wristPID.enable();
        wristPID.setSetpoint(pos);
    }

    public double getSetpoint() {
        return wristPID.getSetpoint();
    }

    public void override(double speed) {
        if(wristPID.isEnabled()) {
            wristPID.disable();
        }
        wrist.set(speed);
    }

    public void stopWrist() {
        override(0);
    }

    public void reset() {
        wristPID.reset();
    }

    public AnalogPotentiometer getPot() {
        return wristPot;
    }
    
    public boolean isPIDEnabled() {
        return wristPID.isEnabled();
    }
}
