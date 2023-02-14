package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class DriveTrain extends SubsystemBase {
//   private final CANSparkMax frontLeft;
//   private final CANSparkMax frontRight;
//   private final CANSparkMax backLeft;
//   private final CANSparkMax backRight;

  private static final int kFrontLeftChannel = 2;
  private static final int kBackLeftChannel = 3;
  private static final int kFrontRightChannel = 1;
  private static final int kBackRightChannel = 0;

  private static final int kJoystickChannel = 0;

  private MecanumDrive m_robotDrive;
  private Joystick m_stick;

  public DriveTrain() {
    CANSparkMax frontLeft = new CANSparkMax(kFrontLeftChannel,MotorType.kBrushless);
    CANSparkMax backLeft = new CANSparkMax(kBackLeftChannel,MotorType.kBrushless);
    CANSparkMax frontRight = new CANSparkMax(kFrontRightChannel,MotorType.kBrushless);
    CANSparkMax backRight = new CANSparkMax(kBackRightChannel,MotorType.kBrushless);

    // Invert the right side motors.
    // You may need to change or remove this to match your robot.
    frontRight.setInverted(true);
    backRight.setInverted(true);

    m_robotDrive = new MecanumDrive(frontLeft, backLeft, frontRight, backRight);

    m_stick = new Joystick(kJoystickChannel);

  }




  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_robotDrive.driveCartesian(-m_stick.getY(), -m_stick.getX(), -m_stick.getZ());
  }
}
