package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake extends TeleopModule{
    CANSparkMax intakeActuator;
    RelativeEncoder intakeEncoder;

    double intakePosition;

    DoubleSolenoid intake;
    DoubleSolenoid piston2;
    
    public Intake() {
        intakeActuator = new CANSparkMax(RobotMap.INTAKE_ACTUATOR, MotorType.kBrushless);
        intakeEncoder = intakeActuator.getEncoder();
        intakeActuator.setInverted(true);
        intake = new DoubleSolenoid (PneumaticsModuleType.CTREPCM, 1, 0);
        //piston1 = new DoubleSolenoid (3, null, 2, 4);
    }

    @Override
    public void teleopControl() {
        if(ControlSystems.getInstance().gamepadB()) {
            intake.set(Value.kForward);
        } else if (ControlSystems.getInstance().gamepadA()) {
            intake.set(Value.kReverse);
        } else {
            intake.set(Value.kOff);
        }
        
        if(ControlSystems.getInstance().gamepadRightY() < -0.1 || ControlSystems.getInstance().gamepadRightY() > 0.1) {
            intakeActuator.set(ControlSystems.getInstance().gamepadRightY()*0.4);
        }
        else if(intakePosition < -7 && ControlSystems.getInstance().gamepadX()) {
            intakeActuator.set(0.4);
        } else if (intakePosition > -5 && ControlSystems.getInstance().gamepadX()) {
            intakeActuator.set(-0.4);
        }
        else if(intakePosition < -60 && ControlSystems.getInstance().gamepadY()) {
            intakeActuator.set(0.4);
        } else if (intakePosition > -58 && ControlSystems.getInstance().gamepadY()) {
            intakeActuator.set(-0.6);
        }
        
        
        else {
            intakeActuator.set(0);
        } 
        
        
        intakePosition = intakeEncoder.getPosition();
        SmartDashboard.putNumber("Intake Position", intakePosition);
    }

    @Override
    public void teleopInit() {
        // TODO Auto-generated method stub
        
    }
    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        
    }
    @Override
    public void robotInit() {
        
    }
}