package frc.robot.subsystems;

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
import frc.robot.commands.pistonIntakeGrab;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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
  final DigitalInput limitSwitch = new DigitalInput(Constants.LimitSwitchDIO);
  
  private void digitalInput() {
    if(limitSwitch.get() == true) {
    intakeActuator.set(0);
  }
  
  SmartDashboard.putBoolean("LimitSwitch: ", limitSwitch.get());

}
public boolean getLimitSwitchBoolean() {
  return SmartDashboard.putBoolean("LimitSwitch: ", limitSwitch.get());
}

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("LimitSwitch: ", limitSwitch.get());
  }






  public void joystickControl() {

    SmartDashboard.putBoolean("LimitSwitch: ", limitSwitch.get());

    
    //Opening and closing intake
    boolean xButton = operatorController.getRawButton(3);
    boolean bButton = operatorController.getRawButton(2);

    //Transfering up and down
    double rightStickY = -operatorController.getRawAxis(5);


    if(xButton) {
      intake.set(Value.kForward);
    } 
    else if (bButton) {
      intake.set(Value.kReverse);
    } 
    else {
      intake.set(Value.kOff);
    }
    if(rightStickY < 0.1) {
      intakeActuator.set(rightStickY/2);
    }
    else if(limitSwitch.get() == false && rightStickY > 0.1) {
      intakeActuator.stopMotor();
    }
    else if((rightStickY < -0.1 || rightStickY > 0.1) && limitSwitch.get() == true) {
      intakeActuator.set(rightStickY/2);
    } 
    else {
      intakeActuator.stopMotor();
    }
    
    SmartDashboard.putNumber("Right Stick Y", rightStickY);

  
  intakePosition = intakeEncoder.getPosition();
  SmartDashboard.putNumber("Intake Position", intakePosition);
  }
  public void intakeUp() {
    while(limitSwitch.get() == true && operatorController.getRawButtonPressed(9)) {
      intakeActuator.set(.5);
    }
  }
  public void intakeDown() {
    intakeEncoder.setPosition(30); //TODO: add a value fr
  }
  public void intakeOut() {
    intake.set(Value.kReverse);
  }
  public void intakeIn() {
    intake.set(Value.kForward);
  }
}



