package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

import java.util.ResourceBundle.Control;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class DriveTrain extends TeleopModule{  
    private double speed;
    private double strafe;
    private double turn;
    private double leftFront;
    private double rightFront;
    private double leftRear;
    private double rightRear;

    private final MotorController frontLeftMotor = new CANSparkMax(1, MotorType.kBrushless);
    private final MotorController frontRightMotor = new CANSparkMax(2, MotorType.kBrushless);
    private final MotorController backLeftMotor = new CANSparkMax(4, MotorType.kBrushless);
    private final MotorController backRightMotor = new CANSparkMax(3, MotorType.kBrushless);

    public DriveTrain() {
    }

    @Override
    public void robotInit() {
        frontRightMotor.setInverted(true);
        backRightMotor.setInverted(true);
    }

    @Override
    public void teleopInit() {
        frontRightMotor.setInverted(true);
        backRightMotor.setInverted(true);
    }

    @Override
    public void teleopControl() {
        speed = -ControlSystems.getInstance().joystickY();
        strafe = ControlSystems.getInstance().joystickX();
        turn = ControlSystems.getInstance().joystickTwist();
    
        leftFront = speed + turn + strafe;
        rightFront = speed - turn - strafe;
        leftRear = speed + turn - strafe;
        rightRear = speed - turn + strafe;
        
        frontLeftMotor.set(leftFront*0.5);
        frontRightMotor.set(rightFront*0.5);
        backLeftMotor.set(leftRear*0.5);
        backRightMotor.set(rightRear*0.5);
    }

    @Override
    public void periodic() {
        
    }
}
