package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import com.revrobotics.RelativeEncoder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.CommandBase;
<<<<<<< Updated upstream
=======
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
>>>>>>> Stashed changes


public class DriveTrain extends SubsystemBase {
  private static final int FrontLeftChannel = 2;
  private static final int BackLeftChannel = 3;
  private static final int FrontRightChannel = 1;
  private static final int BackRightChannel = 0;
  //private static final int kJoystickChannel = 0;

  public RelativeEncoder frontLeftEncoder;
  public RelativeEncoder frontRightEncoder;
  public RelativeEncoder backLeftEncoder;
  public RelativeEncoder backkRightEncoder;
<<<<<<< Updated upstream

  private MecanumDrive m_robotDrive;
  private Joystick m_stick;
=======
  XboxController driverController;



  public void Controllers() {
    driverController = new XboxController(1);



  }  


  private MecanumDrive m_robotDrive;
  




>>>>>>> Stashed changes

  CANSparkMax frontLeft = new CANSparkMax(FrontLeftChannel,MotorType.kBrushless);
  CANSparkMax backLeft = new CANSparkMax(BackLeftChannel,MotorType.kBrushless);
  CANSparkMax frontRight = new CANSparkMax(FrontRightChannel,MotorType.kBrushless);
  CANSparkMax backRight = new CANSparkMax(BackRightChannel,MotorType.kBrushless);

  public DriveTrain() {
    


    frontRight.setInverted(true);
    backRight.setInverted(true);

	//  Invert Left if robots Backwards


    m_robotDrive = new MecanumDrive(frontLeft, backLeft, frontRight, backRight);
    //m_stick = new Joystick(kJoystickChannel);

  }

<<<<<<< Updated upstream

  public CommandBase joystickDrive(double y,double x,double z) {

    double axisValueY = m_stick.getRawAxis(1); //Up and down on lleft Joystick
    double axisValueX = m_stick.getRawAxis(0); //Left RIght on left joystick
    double axisValueZ = m_stick.getRawAxis(4); //X value on right joystick (for turnining)

    //return run(() -> m_robotDrive.driveCartesian(axisValueX,axisValueY,axisValueZ)).withName("Joystick Drive");
    
    return run(() -> m_robotDrive.driveCartesian(x,y,z)).withName("Joystick Drive");
=======
  public boolean joystickBoolean() {

    if(driverController.getRawAxis(0) >= 0.2 || driverController.getRawAxis(0) <= -0.2) {
      return true;
    }
    else if(driverController.getRawAxis(1) >= 0.2 || driverController.getRawAxis(1) <= -0.2  ) {
      return true;
    }
    else if(driverController.getRawAxis(4) >= 0.2 || driverController.getRawAxis(4) <= -0.2  ){
      return true;
    }
    else{
      return false;
    }
  }

  public void joystickDrive() {

    double axisValueX = driverController.getRawAxis(0); //Left RIght on left joystick
    double axisValueY = driverController.getRawAxis(1); //Up and down on lleft Joystick
    double axisValueZ = driverController.getRawAxis(4); //X value on right joystick (for turnining)

    //return run(() -> m_robotDrive.driveCartesian(axisValueX,axisValueY,axisValueZ)).withName("Joystick Drive");
    
    m_robotDrive.driveCartesian(axisValueX,axisValueY,axisValueZ);
>>>>>>> Stashed changes
  
  }






  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //m_robotDrive.driveCartesian(-m_stick.getY(), -m_stick.getX(), -m_stick.getZ());
  }

}
