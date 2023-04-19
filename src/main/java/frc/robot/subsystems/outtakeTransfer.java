package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class outtakeTransfer extends SubsystemBase {
  public CANSparkMax wheelieSpinner;
  public CANSparkMax outtakeMotor;
  double direction;
  private final XboxController operatorController = new XboxController(1);
  double posHold;
  double bumper;

  public outtakeTransfer() {
    outtakeMotor = new CANSparkMax(17, MotorType.kBrushless);
    wheelieSpinner = new CANSparkMax(18, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
  }

  public void wheelieMotorSet(double power) {
    outtakeMotor.set(power);
  }

  public void EncoderPosition() {
    outtakeMotor.getEncoder().setPositionConversionFactor(1.0);
    outtakeMotor.getEncoder().setVelocityConversionFactor(1.0);
    double encoderCount = outtakeMotor.getEncoder().getPosition();
    SmartDashboard.putNumber("Encoder Count", encoderCount);
  }

  public void EncoderPositionReturn() {
    outtakeMotor.getEncoder().setPositionConversionFactor(1.0);
    outtakeMotor.getEncoder().setVelocityConversionFactor(1.0);
    double posHold = outtakeMotor.getEncoder().getPosition();
    SmartDashboard.putNumber("Pos Hold!", posHold);

    //SmartDashboard.putNumber("Encoder Count", encoderCount);
  }

  public void wheelieSetSpeedOpen() {
    SmartDashboard.putNumber("Pos Hold!", posHold);
    while(outtakeMotor.getEncoder().getPosition() - posHold < 9.5) {
      outtakeMotor.set(.35);
    }
    outtakeMotor.set(0);
  }

  public void wheelieSetSpeedClose() {
    SmartDashboard.putNumber("Pos Hold!", posHold);
    while(outtakeMotor.getEncoder().getPosition() - posHold > 0.5) {
      outtakeMotor.set(-.25);
    }
    outtakeMotor.set(0);
  }

  public void wheelMovement() {
    
    boolean yButton = operatorController.getRawButton(4);
    boolean aButton = operatorController.getRawButton(1);

    EncoderPosition();
    direction = operatorController.getPOV(0);
    bumper = operatorController.getRawAxis(2);
    SmartDashboard.putNumber("Back bumpah Numbah", operatorController.getRawAxis(2));

    if (operatorController.getRawAxis(2) > 0.2 || operatorController.getRawAxis(2) < -0.2) {
      outtakeMotor.set(operatorController.getRawAxis(2)/4);
    }
    else if (operatorController.getRawAxis(3) > 0.2 || operatorController.getRawAxis(3) < -0.2) {
      outtakeMotor.set(-operatorController.getRawAxis(3)/4);
    }    
    else if(direction == 0) {
      //outtakeMotor.set(-.15);
      wheelieSetSpeedClose();
    } 
    else if(direction == 180) {
      //outtakeMotor.set(.15);
      wheelieSetSpeedOpen();
    } 
    else {
      outtakeMotor.set(0);
    }



    if(operatorController.getRawButton(1) == true) {
      wheelieSpinner.set(0.3);
    } 
    else if(operatorController.getRawButton(4) == true) {
      wheelieSpinner.set(-.3);
    } 
    else {
      wheelieSpinner.set(0);
    }
  }
}

