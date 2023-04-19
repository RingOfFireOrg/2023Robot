package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OIConstants;

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
  public double encoderPositionHold;
  //Encoder encoderTest = new Encoder(0, 1);
  private final XboxController operatorController = new XboxController(OIConstants.kOperatorControllerPort);



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
    // 3
    encoderPositionHold = encoder.getDistance();
  }

  public void highCubeHeight() {
    //boolean rBumper = operatorController.getRawButton(6);
    encoderPosition = encoder.getDistance();
    while (encoderPosition - encoderPositionHold <= 38.1) {
      extender.set(-1);
      encoderPosition = encoder.getDistance();
    }
    extender.set(0);
  }
  public void midCubeHeight() {
    //    WE NEED TO TEST NUMEBERS!!!!!!!!!!!
    encoderPosition = encoder.getDistance();
    while (encoderPosition - encoderPositionHold <= 15) {
      extender.set(-1);
      encoderPosition = encoder.getDistance();
    }
    extender.set(0);
  }
  public void resetHeight() {
    //boolean rBumper = operatorController.getRawButton(6);
    encoderPosition = encoder.getDistance();
    while (encoderPosition - encoderPositionHold >= 0) {
      extender.set(1);
      encoderPosition = encoder.getDistance();
    }
    extender.set(0);
  }



  public boolean highCubeHeightReturn() {
    encoderPosition = encoder.getDistance();
    while (encoderPosition - encoderPositionHold <= 38.15) {
      extender.set(-1);
      encoderPosition = encoder.getDistance();
    }
    extender.set(0);
    return true;
  }
  
  public boolean midCubeHeightReturn() {
    encoderPosition = encoder.getDistance();
    while (encoderPosition - encoderPositionHold <= 25) {
      extender.set(-1/1.1);
      encoderPosition = encoder.getDistance();
    }
    extender.set(0);
    return true;
  }
  public boolean resetHeightReturn() {
    encoderPosition = encoder.getDistance();
    while (encoderPosition - encoderPositionHold >= 0) {
      extender.set(0.85);
      encoderPosition = encoder.getDistance();
    }
    extender.set(0);
    return true;
  }
















  public void commandOrder() {
    boolean rBumper = operatorController.getRawButton(6);
    if (rBumper == true) {
      highCubeHeight();
    }
    else {
      armMovement();
    }
  }


  public void armMovement() {
    
    double stickVal = operatorController.getRawAxis(1);
    boolean LBumper = operatorController.getRawButton(5);

    encoderPosition = encoder.getDistance();
    SmartDashboard.putNumber("Encoeder Value", encoderPosition);
    SmartDashboard.putNumber("Encoeder Updated Value", encoderPosition - encoderPositionHold);


    if(stickVal < -0.1) {// test to make sure the numbers work
      //moving up
      if(encoder.getDistance() > 38.15) {
        extender.set(0);
      } else {
        extender.set(stickVal/1.4);
      }
      
      //encoderPosition = encoder.getDistance();
    } 
    else if(stickVal > 0.1) {
      //moving down
      extender.set(stickVal/1.8);
      //encoderPosition = encoder.getDistance();

    }
    else {
      extender.set(0);
      //encoderPosition = encoder.getDistance();

    } 
    encoderPosition = encoder.getDistance();

    
  }

}
