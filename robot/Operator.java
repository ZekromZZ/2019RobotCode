package robot;

import robot.subsystems.*;

public class Operator {
    public static double ARM_TOP_ROCKET_BALL;
    public static double ARM_MID_ROCKET_BALL;
    public static double ARM_BOT_ROCKET_BALL;

    public static double WRIST_TOP_ROCKET_BALL;
    public static double WRIST_MID_ROCKET_BALL;
    public static double WRIST_BOT_ROCKET_BALL;
    
    public static double ARM_MID_ROCKET_HATCH;
    public static double ARM_LOW_HATCH;
    
    public static double WRIST_MID_ROCKET_HATCH;
    public static double WRIST_LOW_HATCH;
    
    public static double ARM_CARGO_BALL;
    
    public static double WRIST_CARGO_BALL;
    
    public static double ARM_GND_HATCH;
    public static double WRIST_GND_HATCH;
    
    public static double ARM_STARTUP;
    public static double WRIST_STARTUP;

    // Range of arm and wrist
    public static double WRIST_LOW_RANGE;
    public static double WRIST_HIGH_RANGE;

    public static double ARM_LOW_RANGE;
    public static double ARM_HIGH_RANGE;

    // Tolerance for limiting override range
    public static final double OVERRIDE_TOLERANCE = 2; //Should always be positive

    //Speeds of wheels - Negative speed push ball out
    public static final double WHEELS_SPEED_IN = 0.7;
    public static final double WHEELS_SPEED_OUT = -0.7;

    public static final int PID_ADJUST_SCALE = 10;

    public static final double WRIST_OVERRIDE_SPEED = 0.5;
    public static final double ARM_OVERRIDE_SPEED = 0.5;

    Controller op;
    Arm arm;
    HatchMechanism hatch;
    Wheels wheels;
    Wrist wrist;

    public Operator(int port) {
        op = new Controller(port);
        arm = new Arm();
        hatch = new HatchMechanism();
        wheels = new Wheels();
        wrist = new Wrist();

        ARM_STARTUP = getArmAngle();
        WRIST_STARTUP = getWristAngle();

        ARM_TOP_ROCKET_BALL = ARM_STARTUP + 989;
        ARM_MID_ROCKET_BALL = ARM_STARTUP + 420;
        ARM_BOT_ROCKET_BALL = ARM_STARTUP + 200;

        ARM_MID_ROCKET_HATCH = ARM_STARTUP + 450;
        ARM_LOW_HATCH = ARM_STARTUP + 200;

        ARM_CARGO_BALL = ARM_STARTUP + 765;
        ARM_GND_HATCH = ARM_STARTUP + 313;
        
        ARM_LOW_RANGE = ARM_STARTUP;
        ARM_HIGH_RANGE = ARM_TOP_ROCKET_BALL;

        WRIST_TOP_ROCKET_BALL = WRIST_STARTUP - 1548;
        WRIST_MID_ROCKET_BALL = WRIST_STARTUP - 816;
        WRIST_BOT_ROCKET_BALL = WRIST_STARTUP - 630;

        WRIST_MID_ROCKET_HATCH = WRIST_STARTUP - 450;
        WRIST_LOW_HATCH = WRIST_STARTUP - 65;

        WRIST_CARGO_BALL = WRIST_STARTUP - 1782;
        WRIST_GND_HATCH = WRIST_STARTUP - 1645;

        WRIST_LOW_RANGE = WRIST_CARGO_BALL;
        WRIST_HIGH_RANGE = WRIST_STARTUP;

    }

    public void runOpControls() {
        wheelControl();
        armWristOverride();
        armWristControl();
        hatchControl();

        //System.out.println(wrist.pidOutput());
        System.out.println("Wrist position "+ wrist.getPot().get());
        System.out.println("Arm position "+ arm.getPot().get());

    }

    public void resetPID() {
        arm.reset();
        wrist.reset();
    }

    public double getArmAngle() {
        return arm.getPot().get();
    }

    public double getWristAngle() {
        return wrist.getPot().get();
    }

    private void wheelControl() {
        if (getWristAngle() > (WRIST_STARTUP - OVERRIDE_TOLERANCE)) {
            if(op.getRightBumper() || op.getLeftBumper())
                System.out.println("Wheels cannot spin as the wrist is too close to the arm");
            
            wheels.stopWheels();

            return; //Exits from the function before the speed can be set
        }

        if (op.getRightBumper()) {
            wheels.runWheels(WHEELS_SPEED_IN);
        }
        else if (op.getLeftBumper()) {
            wheels.runWheels(WHEELS_SPEED_OUT);
        }
        else {
            wheels.stopWheels();
        }
    }

    private void armWristOverride() {
        // Wrist override controlled by right stick
        if (op.getRightStickButton()) {
            if ((op.getRightYAxis() < 0) && (getWristAngle() < (WRIST_HIGH_RANGE - OVERRIDE_TOLERANCE))) {
                wrist.override(-op.getRightYAxis() * WRIST_OVERRIDE_SPEED);
                System.out.println("Wrist going up");
            }
            else if ((op.getRightYAxis() > 0) && (getWristAngle() > (WRIST_LOW_RANGE + OVERRIDE_TOLERANCE))) {
                wrist.override(-op.getRightYAxis() * WRIST_OVERRIDE_SPEED);
                System.out.println("Wrist going down");
            }
            else {
                wrist.stopWrist();
                System.out.println("WRIST AT EDGE OF OVERRIDE RANGE");
            }
        }
        else {
            if (!wrist.isPIDEnabled()) {
                wrist.stopWrist();
            }
            else {
                if(Math.abs(op.getRightYAxis())>0.1) {
                    double newWristSetpoint = wrist.getSetpoint() + (-op.getRightYAxis())*PID_ADJUST_SCALE;
                    if(-op.getRightYAxis() > 0 && newWristSetpoint < (WRIST_HIGH_RANGE - OVERRIDE_TOLERANCE)) {
                        wrist.setPosition(newWristSetpoint);
                    }
                    else if(-op.getRightYAxis() < 0 && newWristSetpoint > (WRIST_LOW_RANGE + OVERRIDE_TOLERANCE)) {
                        wrist.setPosition(newWristSetpoint);
                    }
                }
            }
        }

        // Arm override controlled by left stick
        double leftYAxis = -op.getLeftYAxis();
        if (op.getLeftStickButton()) {
            if (((leftYAxis > 0) && (getArmAngle() < (ARM_HIGH_RANGE - OVERRIDE_TOLERANCE)))) {
                arm.override(leftYAxis * ARM_OVERRIDE_SPEED);
                System.out.println("Arm going up");
            }
            else if ((op.getLeftYAxis() < 0) && (getArmAngle() > (ARM_LOW_RANGE + OVERRIDE_TOLERANCE))) {
                arm.override(leftYAxis * ARM_OVERRIDE_SPEED);
                System.out.println("Arm going down");
            }
            else {
                arm.stopArm();
                System.out.println("ARM AT EDGE OF OVERRIDE RANGE");
            }
        }
        else {
            if (!arm.isPIDEnabled()) {
                arm.stopArm();
            }
            else {
                if(Math.abs(leftYAxis)>0.1) {
                    double newArmSetpoint = arm.getSetpoint() + leftYAxis*PID_ADJUST_SCALE;
                    if(leftYAxis > 0 && newArmSetpoint < (ARM_HIGH_RANGE - OVERRIDE_TOLERANCE)) {
                        arm.setPosition(newArmSetpoint);
                    }
                    else if(leftYAxis < 0 && newArmSetpoint > (ARM_LOW_RANGE + OVERRIDE_TOLERANCE)) {
                        arm.setPosition(newArmSetpoint);
                    }
                }
            }
        }
    }

    
//up dpad is no good
//x dpad is bad
    private void armWristControl() {
        if (op.getOButton()) {
            arm.setPosition(ARM_MID_ROCKET_HATCH);
            wrist.setPosition(WRIST_MID_ROCKET_HATCH);
        }
        else if (op.getXButton()) {
            arm.setPosition(ARM_LOW_HATCH);
            wrist.setPosition(WRIST_LOW_HATCH); 
            
        }
        else if (op.getSquareButton()) {
            arm.setPosition(ARM_STARTUP);
            wrist.setPosition(WRIST_STARTUP);
        }
        else if (op.getDPadUp()) {
            arm.setPosition(ARM_TOP_ROCKET_BALL);
            wrist.setPosition(WRIST_TOP_ROCKET_BALL);
        }
        else if (op.getDPadLeft()) {
            arm.setPosition(ARM_MID_ROCKET_BALL);
            wrist.setPosition(WRIST_MID_ROCKET_BALL);
        }
        else if (op.getDPadDown()) {
            arm.setPosition(ARM_BOT_ROCKET_BALL);
            wrist.setPosition(WRIST_BOT_ROCKET_BALL);
        }
        else if (op.getDPadRight()) {
            arm.setPosition(ARM_CARGO_BALL);
            wrist.setPosition(WRIST_CARGO_BALL);
        }
    }

    private void hatchControl() {
        if (op.getTriangleButton()) {
            hatch.place();
        }
        else {
            hatch.retract();
        }
    }
  
    public Controller getController() {
        return op;
    }
}
