package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public class Intake extends TeleopModule{
    MotorController intakeActuator;

    DoubleSolenoid intake;
    DoubleSolenoid piston2;
    
    public Intake() {
        intakeActuator = new CANSparkMax(RobotMap.INTAKE_ACTUATOR, MotorType.kBrushless);

        intake = new DoubleSolenoid (PneumaticsModuleType.CTREPCM, 1, 0);
        //piston1 = new DoubleSolenoid (3, null, 2, 4);
    }

    @Override
    public void teleopControl() {
        if(ControlSystems.getInstance().gamepadA()) {
            intake.set(Value.kForward);
        } else if (ControlSystems.getInstance().gamepadB()) {
            intake.set(Value.kReverse);
        } else {
            intake.set(Value.kOff);
        }
        
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