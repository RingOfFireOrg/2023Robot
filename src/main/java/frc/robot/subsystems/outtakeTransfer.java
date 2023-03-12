package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class outtakeTransfer extends SubsystemBase {
  VictorSP wheelieSpinner;
  CANSparkMax outtakeMotor;
  double direction;
  private final XboxController operatorController = new XboxController(1);

  public outtakeTransfer() {
    outtakeMotor = new CANSparkMax(17, MotorType.kBrushless);
    wheelieSpinner = new VictorSP(0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void wheelMovement() {

    direction = operatorController.getPOV(0);
    
    if(direction == 0) {
      outtakeMotor.set(-.15);
    } 

    else if(direction == 180) {
      outtakeMotor.set(.15);
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
