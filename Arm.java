package robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

public class Arm {

    private final int potRange = 2000;//Range of pot in degrees
    private final int potOffset = -1000;//Offset of pot from level

    private final double p = 0.0013;//0.85
    private final double i = 0;//0.01
    private final double d = 0;//0

    private final double maxSpeed = 1;

    private WPI_TalonSRX left;
    private WPI_TalonSRX right;

    private SpeedControllerGroup armGroup; 

    private AnalogPotentiometer armPot;

    private PIDController armPID;

    public Arm() {
        left = new WPI_TalonSRX(4);
        right = new WPI_TalonSRX(5);

        right.setInverted(true);//Inverting one motor as they drive in different directions. CHECK WHICH SIDE TO INVERT.

        armGroup = new SpeedControllerGroup(left, right);

        armPot = new AnalogPotentiometer(0, potRange, potOffset);

        armPID = new PIDController(p, i, d, armPot, armGroup);
        armPID.setInputRange(potOffset, potRange+potOffset);
        armPID.setOutputRange(-maxSpeed, maxSpeed);
    }

    public double pidOutput() {
        return armPID.get();
    }

    public void setPosition(double pos) {
        armPID.enable();
        armPID.setSetpoint(pos);
    }

    public void override(double speed) {
        armPID.disable();
        armGroup.set(speed);
    }

    public void stopArm() {
        override(0);
    }

    public void reset() {
        armPID.reset();
    }

    public AnalogPotentiometer getPot() {
        return armPot;
    }
    
    public boolean isPIDEnabled() {
        return armPID.isEnabled();
    }
}
