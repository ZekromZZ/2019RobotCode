package robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in 
 * the project.
 */
public class Robot extends TimedRobot {

    public static Driver drive;
    public static Operator op;

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
        //Controller init
        drive = new Driver(0);
        op = new Operator(1);

        //Camera init
        CameraServer.getInstance().startAutomaticCapture();
    }

    /**
     * This function is called when the disabled button is hit.
     * You can use it to reset subsystems before shutting down.
     */
    @Override
    public void disabledInit(){
    }

    @Override
    public void disabledPeriodic() {
        Scheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
        teleopInit();
    }

    /**
     * This function is called periodically during autonomous
     */
    @Override
    public void autonomousPeriodic() {
        Scheduler.getInstance().run();
        
        teleopPeriodic();
    }

    @Override
    public void teleopInit() {
        op.resetPID();
    }

    /**
     * This function is called periodically during operator control
     */
    @Override
    public void teleopPeriodic() {
        Scheduler.getInstance().run();

        drive.runDriveControls();
        op.runOpControls();
    }

    @Override
    public void testInit() {
        op.resetPID();
    }

    @Override
    public void testPeriodic() {
      // Scheduler.getInstance().run();
      // System.out.println("Wrist angle: " + op.getWristAngle());
      // System.out.println("Arm angle: " + op.getArmAngle());
    }
}