package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
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
  }


  public void armMovement(Supplier<Double> gamepadRightYValue, Supplier<Boolean> aButton, Supplier<Boolean> bButton) {
    
    double stickVal = operatorController.getRawAxis(1);
    boolean aButton1 = operatorController.getRawButton(4);
    boolean bButton1 = operatorController.getRawButton(5);

    encoderPosition = encoder.getDistance();
    SmartDashboard.putNumber("Encoeder Value", encoderPosition);

    if((stickVal < -0.1 || stickVal > 0.1)) {
      extender.set(stickVal/2);
      encoderPosition = encoder.getDistance();
    } 
    // else if(encoderPosition > 44 && aButton1 == true) {
    //   extender.set(-.2);
    //   encoderPosition = encoder.getDistance();

    // } 
    // else if(encoderPosition < 42 && aButton1 == true) {
    //   extender.set(.8);
    //   encoderPosition = encoder.getDistance();

    // }
    // else if(encoderPosition < 4 && bButton1 == true) {
    //   extender.set(.3);
    //   encoderPosition = encoder.getDistance();

    // }
    // else if(encoderPosition > 6 && bButton1 == true) {
    //   extender.set(-0.4);
    //   encoderPosition = encoder.getDistance();

    // }
    else {
      extender.set(0);
      encoderPosition = encoder.getDistance();

    } 
    encoderPosition = encoder.getDistance();

 
    
  }
}
