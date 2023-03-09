package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;

public class pistonIntake extends SubsystemBase {
  CANSparkMax intakeActuator;
  RelativeEncoder intakeEncoder;

  double intakePosition;

  DoubleSolenoid intake;
  DoubleSolenoid piston2;
  
  private final XboxController operatorController = new XboxController(1);
 
  public pistonIntake() {
    intakeActuator = new CANSparkMax(16, MotorType.kBrushless);
    intakeEncoder = intakeActuator.getEncoder();

    intakeActuator.setInverted(true);
    intake = new DoubleSolenoid (PneumaticsModuleType.CTREPCM, 1, 0);
  }
  //final DigitalInput limitSwitch = new DigitalInput(Constants.LimitSwitchDIO);
  
  // private void digitalInput() {
  //   if(limitSwitch.get() == true) {
  //   intakeActuator.set(0);
  // }
  
//   SmartDashboard.putBoolean("LimitSwitch: ", limitSwitch.get());

// }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }






  public void joystickControl() {


    
    //Opening and closing intake
    boolean xButton = operatorController.getRawButton(3);
    boolean bButton = operatorController.getRawButton(2);

    //Transfering up and down
    double rightStickY = operatorController.getRawAxis(5);


    if(xButton) {
      intake.set(Value.kForward);
    } 
    else if (bButton) {
      intake.set(Value.kReverse);
    } 
    else {
      intake.set(Value.kOff);
    }
    // if(limitSwitch.get() == true) {
    //   intakeActuator.stopMotor();
    // }
    if((rightStickY < -0.1 || rightStickY > 0.1)/* && limitSwitch.get() == false*/) {
      intakeActuator.set(rightStickY/2);
    } 
    else {
      intakeActuator.stopMotor();
    }
    
    SmartDashboard.putNumber("Right Stick Y", rightStickY);
    //SmartDashboard.putNumber("intake encoder: ", intakeEncoder);
  
    // else if(intakePosition < -7 && xButton) {
    //   intakeActuator.set(0.4);
    // } 
    // else if (intakePosition > -5 && xButton) {
    //   intakeActuator.set(-0.4);
    // }
    // else if(intakePosition < -60 && yButton) {
    //   intakeActuator.set(0.4);
    // } 
    // else if (intakePosition > -58 && yButton) {
    //   intakeActuator.set(-0.6);
    // }

  
  
  intakePosition = intakeEncoder.getPosition();
  SmartDashboard.putNumber("Intake Position", intakePosition);
  }


}



