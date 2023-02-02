package frc.robot;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public class Extender extends TeleopModule{
   
    public CANSparkMax extender;
    public RelativeEncoder extenderEncoder;
    public double extenderPosition;


    public Extender () {
        extender = new CANSparkMax(RobotMap.LINEAR_SLIDE, MotorType.kBrushless);
        extenderEncoder = extender.getEncoder();
    }

    public void robotInit() {
    }

    public void teleopInit() {
    }

    public void teleopControl() {
        extender.set(ControlSystems.getInstance().gamepadLeftY());
        extenderPosition = extenderEncoder.getPosition();
        SmartDashboard.putNumber("Extender Position", extenderPosition);
    }

    public void periodic() {
    }
}