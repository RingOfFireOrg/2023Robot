package frc.robot;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;



public class Extender extends TeleopModule{
    public CANSparkMax extenderMotor1;
    public CANSparkMax extenderMotor2;
    public RelativeEncoder extenderEncoder1;
    public RelativeEncoder extenderEncoder2;
    public double extender1Position;
    public double extender2Position;
    public DutyCycleEncoder encoder;
    public double encoderPosition;
              
    public Extender () {
        extenderMotor1 = new CANSparkMax(14, MotorType.kBrushless);
        extenderMotor2 = new CANSparkMax(15, MotorType.kBrushless);
        encoder = new DutyCycleEncoder(0);
        encoder.setDistancePerRotation(4.0);
    }

    public void robotInit() {
    }

    public void teleopInit() {
    }

    public void teleopControl() {
        extenderMotor2.set(ControlSystems.getInstance().gamepadLeftY());
        extenderMotor1.set(ControlSystems.getInstance().gamepadRightY());
        encoderPosition = encoder.getDistance();

        SmartDashboard.putNumber("Extender Position: ", encoderPosition);
    }
    public void periodic() {
    }
}