package frc.robot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

public class Controllers {
    
    private static Controllers thetrueControlSystem;
    Joystick manipulatorStick;
    XboxController driverController;
    XboxController manipulatorController;
    public Controllers() {
        manipulatorController = new XboxController(2);
        driverController = new XboxController(3);
    }

    //Driver controller functions
    public double dGamepadLeftY() {
        return driverController.getRawAxis(1);
    }
    public double dGamepadRightY() {
        return driverController.getRawAxis(5);
    }
    public boolean dGamepadA() {
        return driverController.getRawButton(1);
    }
    public boolean dGamepadB() {
        return driverController.getRawButton(2);
    }
    public boolean dGamepadRightBumper() {
        return driverController.getRawButton(6);
    }
    public boolean dGamepadLeftBumper() {
        return driverController.getRawButton(5);
    }
    public double mGamepadLeftY() {
        return manipulatorController.getRawAxis(1);
    }
    public double mGamepadRightY() {
        return manipulatorController.getRawAxis(5);
    }
    public boolean dGamepadBack() {
        return driverController.getRawButton(7);
    }
    public boolean dGamepadStart() {
        return driverController.getRawButton(8);
    }

    //Manipulator controller functions
    public boolean mGamepadA() {
        return manipulatorController.getRawButton(1);
    }
    public boolean mGamepadB() {
        return manipulatorController.getRawButton(2);
    }
    
    public boolean mGamepadX() {
        return manipulatorController.getRawButton(3);
    }
    public boolean mGamepadY() {
        return manipulatorController.getRawButton(4);
    }
    
    public boolean mGamepadRightBumper() {
        return manipulatorController.getRawButton(6);
    }
    public boolean mGamepadLeftBumper() {
        return manipulatorController.getRawButton(5);
    }
    
    public boolean mGamepadStart() {
        return manipulatorController.getRawButton(8);
    }
    public double mGamepadLeftTrigger() {
        return manipulatorController.getRawAxis(2);
    }
    public double mGamepadRightTrigger() {
        return manipulatorController.getRawAxis(3);
    }
    public String mGamepadPov() {
        if (manipulatorController.getPOV() == 0) {
            return "up";
        } 
        if (manipulatorController.getPOV() == 180) {
            return "down";
        }
        return "";
    }
    public static Controllers get() {
        if (thetrueControlSystem != null) return thetrueControlSystem;

        thetrueControlSystem = new Controllers();
        return thetrueControlSystem;
    }
}