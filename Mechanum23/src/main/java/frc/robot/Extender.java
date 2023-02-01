package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public class Extender extends TeleopModule{
   
    public MotorController extender;


    public Extender () {
        extender = new CANSparkMax(RobotMap.LINEAR_SLIDE, MotorType.kBrushless);
    }

    public void robotInit() {
    }

    public void teleopInit() {
    }

    public void teleopControl() {
        extender.set(ControlSystems.getInstance().gamepadLeftY());
    }

    public void periodic() {
    }
}