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
    XboxController driverController;

    public ControlSystems() {
        driverJoystick = new Joystick(0);
        manipulatorController = new XboxController(1);
        driverController = new XboxController(2);
    }

    // Joystick Controls
    public double joystickY() {
            return driverJoystick.getY();
    }
    public double joystickX() {
        return driverJoystick.getX();
    }
    public double joystickTwist() {
        return driverJoystick.getTwist();
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
    public boolean gamepadY() {
        return manipulatorController.getRawButton(4);
    }
    public boolean gamepadB() {
        return manipulatorController.getRawButton(2);
    }
    public boolean gamepadA() {
        return manipulatorController.getRawButton(1);
    }
    public double dgamepadLeftY() {
        return driverController.getRawAxis(RobotMap.MANIPULATOR_LEFT_JOYSTICK_Y);
    }
    public double dgamepadRightY() {
        return driverController.getRawAxis(RobotMap.MANIPULATOR_RIGHT_JOYSTICK_Y);
    }
    public double dgamepadLeftTrigger() {
        return driverController.getRawAxis(RobotMap.MANIPULATOR_LEFT_TRIGGER);
    }
    public double dgamepadRightTrigger() {
        return driverController.getRawAxis(RobotMap.MANIPULATOR_RIGHT_TRIGGER);
    }
    public boolean dgamepadLB() {
        return driverController.getRawButton(5);
    }
    public boolean dgamepadRB() {
        return driverController.getRawButton(6);
    }
    public boolean dgamepadX() {
        return driverController.getRawButton(3);
    }
    public boolean dgamepadB() {
        return driverController.getRawButton(2);
    }
    public boolean dgamepadA() {
        return driverController.getRawButton(1);
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