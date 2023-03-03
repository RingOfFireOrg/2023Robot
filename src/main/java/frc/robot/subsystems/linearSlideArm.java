package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class linearSlideArm extends SubsystemBase {
  /** Creates a new linearSlideArm. */
  private CANSparkMax extenderMotor1;
  private CANSparkMax extenderMotor2;
  public RelativeEncoder extenderEncoder1;
  public RelativeEncoder extenderEncoder2;
  public double extender1Position;
  public double extender2Position;
  public DutyCycleEncoder encoder;
  public MotorControllerGroup extender;
  public double encoderPosition;



  public linearSlideArm() {
    extenderMotor1 = new CANSparkMax(14, MotorType.kBrushless);
    extenderMotor2 = new CANSparkMax(15, MotorType.kBrushless);
    extenderMotor1.setIdleMode(IdleMode.kBrake);
    extenderMotor2.setIdleMode(IdleMode.kBrake);
    extender = new MotorControllerGroup(extenderMotor1, extenderMotor2);
    encoder = new DutyCycleEncoder(0);
    encoder.setDistancePerRotation(4.0);
    extenderMotor1.setInverted(true);
    extenderMotor2.setInverted(true);

  }

  @Override
  public void periodic() {


  }

  public void encoderReset() {
    encoder.reset();
  }


  public void armMovement(double gamepadRightYValue, boolean aButton, boolean bButton) {
    if(gamepadRightYValue < -0.1 || gamepadRightYValue > 0.1) {
      extender.set(gamepadRightYValue);
    } 
    else if(encoderPosition > 44 && aButton == true) {
      extender.set(-.2);
    }
    else if(encoderPosition < 42 && aButton == true) {
      extender.set(.8);
    }
    else if(encoderPosition < 4 && bButton == true) {
      extender.set(.3);
    }
    else if(encoderPosition > 6 && bButton == true) {
      extender.set(-0.4);
    }
    else {
      extender.set(0);
    } 
    
  }
}
