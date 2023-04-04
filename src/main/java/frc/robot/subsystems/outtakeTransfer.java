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
  VictorSP wheelieSpinner;
  public CANSparkMax outtakeMotor;
  double direction;
  private final XboxController operatorController = new XboxController(1);
  double posHold;

  public outtakeTransfer() {
    outtakeMotor = new CANSparkMax(17, MotorType.kBrushless);
    wheelieSpinner = new VictorSP(0);

  }

  @Override
  public void periodic() {
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
    while(outtakeMotor.getEncoder().getPosition() - posHold < 10.0) {
      outtakeMotor.set(.15);
    }
    outtakeMotor.set(0);
  }

  public void wheelieSetSpeedClose() {
    SmartDashboard.putNumber("Pos Hold!", posHold);
    while(outtakeMotor.getEncoder().getPosition() - posHold > 1.0) {
      outtakeMotor.set(-.15);
    }
    outtakeMotor.set(0);
  }

  public void wheelMovement() {
    EncoderPosition();
    direction = operatorController.getPOV(0);
    
    if(direction == 0) {
      //outtakeMotor.set(-.15);
      wheelieSetSpeedOpen();
    } 

    else if(direction == 180) {
      //outtakeMotor.set(.15);
      wheelieSetSpeedClose();
    } 

    else {
      outtakeMotor.set(0);
    }

    if(direction == 270) {
      wheelieSpinner.set(1);
    } 

    else if(direction == 90) {
      wheelieSpinner.set(-.5);
    } 

    else {
      wheelieSpinner.set(0);
    }
  }
}

