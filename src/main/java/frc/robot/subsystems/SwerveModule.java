package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants;
import frc.robot.Auto.OnboardModuleState;

public class SwerveModule {

    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;
    private Rotation2d lastAngle;
    private Rotation2d angleOffset;

    // private final SparkMaxPIDController driveController;
    // private final SparkMaxPIDController angleController;
    
    private final RelativeEncoder  driveEncoder;
    private final RelativeEncoder  turningEncoder;

    private final PIDController drivingPidController;

    private final PIDController turningPidController;

    private final CANCoder absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;
    private final SwerveModulePosition position = new SwerveModulePosition();
    private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(
        0.743, 2.78, 0.406
);
    

    private int encId;


    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new CANCoder(absoluteEncoderId);

        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();

        //set to coast;
        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);


        //turningEncoder.setIntegratedSensorPosition(absoluteEncoder., timeoutMs)

        encId = absoluteEncoderId;

        CANCoderConfiguration config = new CANCoderConfiguration();
        // set units of the CANCoder to radians, with velocity being radians per second
        config.sensorCoefficient = 2 * Math.PI / 4096.0; //convert to radians
        config.unitString = "rad";
        config.sensorTimeBase = SensorTimeBase.PerSecond;
        absoluteEncoder.configAllSettings(config);

        //driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        ///driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        //turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        drivingPidController = new PIDController(3, 0, 0);
        //drivingPidController.enableContinuousInput(-Math.PI, Math.PI);

        // angleOffset = moduleConstants.angleOffset;

        // /* Angle Encoder Config */
        // angleEncoder = new CANCoder(moduleConstants.cancoderID);
        // configAngleEncoder();
    
        // /* Angle Motor Config */
        // angleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        // integratedAngleEncoder = angleMotor.getEncoder();
        // angleController = angleMotor.getPIDController();
        // configAngleMotor();
    
        // /* Drive Motor Config */
        // driveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
        // driveEncoder = driveMotor.getEncoder();
        // driveController = driveMotor.getPIDController();
        // configDriveMotor();
    
        lastAngle = getState().angle;

        resetEncoders();
    }
    public double getAzimuthPosition() {
        return Math.toRadians(absoluteEncoder.getAbsolutePosition());
    }

    public SwerveModulePosition getPosition() {
        position.distanceMeters = getDrivePosition();
        position.angle = new Rotation2d(getAzimuthPosition());
        return position;
    }
    
    public double getDrivePosition() { //math is for manual conversion factor because TalonFX controllers do not have ConversionFactor functions
        return ((driveEncoder.getPosition() / ModuleConstants.kEncoderCPR) * ModuleConstants.kDriveEncoderRot2Meter);
    }

    public double getTurningPosition() {
        return (getAbsoluteEncoderRad());
    }

    public double getDriveVelocity() {
        return ((driveEncoder.getPosition() / ModuleConstants.kEncoderCPR) * ModuleConstants.kDriveEncoderRPM2MeterPerSec);
    }

    public double getTurningVelocity() {
        return (absoluteEncoder.getVelocity());
    }

    public double getAbsoluteEncoderRad() {
        double angle = absoluteEncoder.getAbsolutePosition();
        //angle = Math.toRadians(angle);
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        SmartDashboard.putNumber("Swerve[" + encId + "] state", getAbsoluteEncoderRad());
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);

        //driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
         driveMotor.set(drivingPidController.calculate(getDrivePosition(), (state.speedMetersPerSecond/ DriveConstants.kPhysicalMaxSpeedMetersPerSecond)));

        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));

    }


    // public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    //     // Custom optimize command, since default WPILib optimize assumes continuous controller which
    //     // REV and CTRE are not
    //     desiredState = OnboardModuleState.optimize(desiredState, getState().angle);
    
    //     setAngle(desiredState);
    //     setSpeed(desiredState, isOpenLoop);
    // }

    public void setDesiredState3(SwerveModuleState state) {
        SmartDashboard.putNumber("Swerve[" + encId + "] state", getAbsoluteEncoderRad());
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);

        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        //driveMotor.set(drivingPidController.calculate(getDrivePosition(), state.speedMetersPerSecond/ DriveConstants.kPhysicalMaxSpeedMetersPerSecond));

        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));

    }



    // public void setDesiredStatee(SwerveModuleState desiredState) {
    //     //double turnRadians = getT//urnRadians();
    
    //     // Optimize the reference state to avoid spinning further than 90 degrees
    //     //SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(desiredState, new Rotation2d(turnRadians));
    
    //     // Converts meters per second to rpm
    //     double desiredDriveRPM = optimizedDesiredState.speedMetersPerSecond * 60 
    //       * ModuleConstants.kDriveMotorGearRatio / ModuleConstants.kWheelDiameterMeters;
          
    //     // Converts rpm to encoder units per 100 milliseconds
    //     double desiredDriveEncoderUnitsPer100MS = desiredDriveRPM / 600.0 * 2048;
    
    //     // Sets the drive motor's speed using the built in pid controller
    //     driveMotor.set(ControlMode.Velocity, desiredDriveEncoderUnitsPer100MS);
    
    //     // Calculate the turning motor output from the turn PID controller.
    //     double turnOutput =
    //       turnPIDController.calculate(turnRadians, optimizedDesiredState.angle.getRadians())
    //         + turnFeedForward.calculate(turnPIDController.getSetpoint().velocity);
    //         turnMotor.set(turnOutput / 12);
    //   }
    // public void setDesiredState(SwerveModuleState desiredState) {
    //     double driveVelocity = getDriveVelocity();
    //     double azimuthPosition = getAzimuthPosition();

    //     desiredState = SwerveModuleState.optimize(desiredState, getState().angle);
    //     if (Math.abs(desiredState.speedMetersPerSecond) < 0.01
    //             && Math.abs(desiredState.angle.getRadians() - azimuthPosition) < 0.05) {
    //         stop();
    //         return;
    //     }

    //     final double driveOutput =
    //             drivingPidController.calculate(driveVelocity, desiredState.speedMetersPerSecond)
    //                     + driveFeedforward.calculate(desiredState.speedMetersPerSecond);

    //     desiredState = SwerveModuleState.optimize(desiredState, getState().angle);

    //     driveMotor.setVoltage(driveOutput);
    //     turningMotor.set(turningPidController.calculate(getTurningPosition(), desiredState.angle.getRadians()));

    //     //turningMotor.setVoltage(azimuthOutput);
    // }




    // public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    //     // Custom optimize command, since default WPILib optimize assumes continuous controller which
    //     // REV and CTRE are not
    //     desiredState = OnboardModuleState.optimize(desiredState, getState().angle);
    
    //     setAngle(desiredState);
    //     setSpeed(desiredState, isOpenLoop);
    // }


    // private void setAngle(SwerveModuleState desiredState) {
    //     // Prevent rotating module if speed is less then 1%. Prevents jittering.
    //     Rotation2d angle =
    //         (Math.abs(desiredState.speedMetersPerSecond) <= (DriveConstants.kPhysicalMaxSpeedMetersPerSecond * 0.01))
    //             ? lastAngle
    //             : desiredState.angle;
    
    //     angleController.setReference(angle.getDegrees(), ControlType.kPosition);
    //     lastAngle = angle;
    // }









    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }

    public void brake(boolean doBrake){
        if(doBrake){
            driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        }
        else{
            driveMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        }
        
    }
}