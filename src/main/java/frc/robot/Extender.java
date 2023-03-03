package frc.robot;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;



public class Extender extends TeleopModule{
    public CANSparkMax extenderMotor1;
    public CANSparkMax extenderMotor2;
    public RelativeEncoder extenderEncoder1;
    public RelativeEncoder extenderEncoder2;
    public double extender1Position;
    public double extender2Position;
    public DutyCycleEncoder encoder;
    public MotorControllerGroup extender;
    public double encoderPosition;
              
    public Extender () {
        extenderMotor1 = new CANSparkMax(14, MotorType.kBrushless);
        extenderMotor2 = new CANSparkMax(15, MotorType.kBrushless);
        extender = new MotorControllerGroup(extenderMotor1, extenderMotor2);
        encoder = new DutyCycleEncoder(0);
        encoder.setDistancePerRotation(4.0);
        extenderMotor1.setInverted(true);
        extenderMotor2.setInverted(true);
    }

    public void robotInit() {
        encoder.reset();
    }

    public void teleopInit() {

    }

    public void teleopControl() {
        if(ControlSystems.getInstance().gamepadRightY() < -0.1 || ControlSystems.getInstance().gamepadRightY() > 0.1) {
            extender.set(ControlSystems.getInstance().gamepadRightY());
        }
        else if(encoderPosition > 44 && ControlSystems.getInstance().gamepadA()) {
            extender.set(-.2);
        }
        else if(encoderPosition < 42 && ControlSystems.getInstance().gamepadA()) {
            extender.set(.8);
        }
        else if(encoderPosition < 4 && ControlSystems.getInstance().gamepadB()) {
            extender.set(.3);
        }
        else if(encoderPosition > 6 && ControlSystems.getInstance().gamepadB()) {
            extender.set(-0.4);
        }
        else {
            extender.set(0);
        } 

        //if(encoderPosition < )

        encoderPosition = encoder.getDistance();
        SmartDashboard.putNumber("Extender Position: ", encoderPosition);
    }


    public void periodic() {
    }
}