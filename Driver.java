package robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import robot.subsystems.*;

public class Driver {

    Controller joy;

    DriveTrain driveTrain;

    private SendableChooser<Byte> driveType;
    private final Byte arcade = 0;
    private final Byte tank = 1;

    public Driver(int port) {
        joy = new Controller(port);
        driveTrain = new DriveTrain();

        driveType = new SendableChooser<>();
        driveType.setDefaultOption("Arcade", arcade);
        driveType.addOption("Tank", tank);
        SmartDashboard.putData("Drive Type", driveType);
    }

    public void runDriveControls() {
        if(driveType.getSelected().equals(arcade))
            driveTrain.arcadeDrive(joy.getRightYAxis(), joy.getLeftTrigger(), joy.getRightTrigger());
        else if(driveType.getSelected().equals(tank))
            driveTrain.tankDrive(joy.getLeftYAxis(), joy.getRightYAxis());
        else
            System.out.println("Error: No drive type chosen");
    }
}