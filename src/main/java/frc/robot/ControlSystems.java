package frc.robot;

import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

public class ControlSystems {
    
    private static ControlSystems thetrueControlSystem;
    Joystick driverJoystick;
    XboxController manipulatorController;

    public ControlSystems() {
        driverJoystick = new Joystick(0);
        manipulatorController = new XboxController(1);
    }

    // Joystick Controls
    public double joystickY() {
        if (Math.abs(driverJoystick.getY()) < .1) {
            return 0;
        }
        return driverJoystick.getY();
    }
    public double joystickX() {
        if (Math.abs(driverJoystick.getX()) < .1) {
            return 0;
        }
        return driverJoystick.getY();
    }
    public double joystickTwist() {
        if (Math.abs(driverJoystick.getTwist()) < .1) {
            return 0;
        }
        return driverJoystick.getY();
    }

    // GamePad Controls
    public double gamepadLeftY() {
        return manipulatorController.getRawAxis(RobotMap.MANIPULATOR_LEFT_JOYSTICK_Y);
    }
    public double gamepadRightY() {
        return manipulatorController.getRawAxis(RobotMap.MANIPULATOR_RIGHT_JOYSTICK_Y);
    }
    public double gamepadLeftTrigger() {
        return manipulatorController.getRawAxis(RobotMap.MANIPULATOR_LEFT_TRIGGER);
    }
    public double gamepadRightTrigger() {
        return manipulatorController.getRawAxis(RobotMap.MANIPULATOR_RIGHT_TRIGGER);
    }
    public boolean gamepadLB() {
        return manipulatorController.getRawButton(5);
    }
    public boolean gamepadRB() {
        return manipulatorController.getRawButton(6);
    }
    public boolean gamepadX() {
        return manipulatorController.getRawButton(3);
    }
    public boolean gamepadB() {
        return manipulatorController.getRawButton(2);
    }
    public boolean gamepadA() {
        return manipulatorController.getRawButton(1);
    }
    public static ControlSystems getInstance() {
        if (thetrueControlSystem != null) {
            return thetrueControlSystem;
        }                   
        else {
            thetrueControlSystem = new ControlSystems();
            return thetrueControlSystem;
        }
    }
}