package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {

    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    private final RelativeEncoder  driveEncoder;
    private final RelativeEncoder  turningEncoder;

    private final PIDController turningPidController;

    private final CANCoder absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;


    //new piddies
    private final PIDController drivePIDController = new PIDController
    (Constants.DriveConstants.DriveMotor_P, Constants.DriveConstants.DriveMotor_I, Constants.DriveConstants.DriveMotor_D);
    
    private final ProfiledPIDController azimuthPIDController = new ProfiledPIDController
    (Constants.DriveConstants.AzimuthMotor_P, Constants.DriveConstants.AzimuthMotor_I, Constants.DriveConstants.AzimuthMotor_D,
    new TrapezoidProfile.Constraints(3 * Math.PI, 6 * Math.PI));
   

    private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(
        DriveConstants.DRIVE_MOTOR_KS, DriveConstants.DRIVE_MOTOR_KV, DriveConstants.DRIVE_MOTOR_KA);
    private final SimpleMotorFeedforward azimuthFeedForward = new SimpleMotorFeedforward(
        DriveConstants.AZIMUTH_MOTOR_KS, DriveConstants.AZIMUTH_MOTOR_KV, DriveConstants.AZIMUTH_MOTOR_KA);
    
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


        resetEncoders();
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
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));

    }



    // setDesiredState using setVoltage
    public void setDesiredStateVoltage(SwerveModuleState desiredState) {
        //double driveVelocity = getDriveVelocity();
        //double azimuthPosition = getAzimuthPosition();
        SmartDashboard.putNumber("Swerve[" + encId + "] state", getAbsoluteEncoderRad());

        double driveVelocity = driveEncoder.getVelocity() * 10 / 2048 * 
        (Constants.ModuleConstants.kWheelDiameterMeters * Math.PI) * Constants.ModuleConstants.kDriveMotorGearRatio;  
        //double azimuthPosition = Math.toRadians(turningEncoder.getPosition());   <-- NOT absolute encoder
        //double azimuthPosition = Math.toRadians(absoluteEncoder.getAbsolutePosition());    <- diff method of getting absolute position
        double azimuthPosition = getAbsoluteEncoderRad();

        desiredState = SwerveModuleState.optimize(desiredState, new Rotation2d(azimuthPosition));

        if (Math.abs(desiredState.speedMetersPerSecond) < 0.01
                && Math.abs(desiredState.angle.getRadians() - azimuthPosition) < 0.05) {
            stop();
            return;
        }

        // this one if a failsafe in case the other one dosent work
        if (Math.abs(desiredState.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }

        final double driveOutput =
                drivePIDController.calculate(driveVelocity, desiredState.speedMetersPerSecond)
                        + driveFeedforward.calculate(desiredState.speedMetersPerSecond);

        final double azimuthOutput =
                azimuthPIDController.calculate(azimuthPosition, desiredState.angle.getRadians())
                        + azimuthFeedForward.calculate(azimuthPIDController.getSetpoint().velocity);


        //Feed Forward Constants are 0 becuase we arent making curves in our path rn so they dont matter
        //If we want to, use sysID to get numbers

        driveMotor.setVoltage(driveOutput);
        turningMotor.setVoltage(azimuthOutput);
    }




    // setDesiredState using setVoltage but only for driving
    public void setDesiredStateVoltagedrive(SwerveModuleState desiredState) {
        //double driveVelocity = getDriveVelocity();
        //double azimuthPosition = getAzimuthPosition();
        SmartDashboard.putNumber("Swerve[" + encId + "] state", getAbsoluteEncoderRad());

        double driveVelocity = driveEncoder.getVelocity() * 10 / 2048 * 
        (Constants.ModuleConstants.kWheelDiameterMeters * Math.PI) * Constants.ModuleConstants.kDriveMotorGearRatio;  
        //double azimuthPosition = Math.toRadians(turningEncoder.getPosition());   <-- NOT absolute encoder
        //double azimuthPosition = Math.toRadians(absoluteEncoder.getAbsolutePosition());    <- diff method of getting absolute position
        double azimuthPosition = getAbsoluteEncoderRad();


        SwerveModuleState state = SwerveModuleState.optimize(desiredState, getState().angle);
        desiredState = SwerveModuleState.optimize(desiredState, new Rotation2d(azimuthPosition));

        if (Math.abs(desiredState.speedMetersPerSecond) < 0.01
                && Math.abs(desiredState.angle.getRadians() - azimuthPosition) < 0.05) {
            stop();
            return;
        }

        // this one if a failsafe in case the other one dosent work
        if (Math.abs(desiredState.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }

        final double driveOutput =
                drivePIDController.calculate(driveVelocity, desiredState.speedMetersPerSecond)
                        + driveFeedforward.calculate(desiredState.speedMetersPerSecond);

        // final double azimuthOutput =
        //         azimuthPIDController.calculate(azimuthPosition, desiredState.angle.getRadians())
        //                 + azimuthFeedForward.calculate(azimuthPIDController.getSetpoint().velocity);


        //Feed Forward Constants are 0 becuase we arent making curves in our path rn so they dont matter
        //If we want to, use sysID to get numbers

        // Azimuth output is working fine currently, so If zimuth dosnet work with setvoltage this one should work
        SmartDashboard.putNumber("Drive Motor Set voltage", driveOutput);
        driveMotor.setVoltage(driveOutput);
        SmartDashboard.putNumber("Turnign Motor set", turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
    }



    // Alternnative instead of just "setvoltage"
    public void setDesiredStateAlternative(SwerveModuleState desiredState) {
        double turnRadians = ((2 * Math.PI) / 360) * absoluteEncoder.getAbsolutePosition();
    
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(desiredState, new Rotation2d(turnRadians));
    
        // Converts meters per second to rpm
        double desiredDriveRPM = optimizedDesiredState.speedMetersPerSecond * 60 
          * Constants.ModuleConstants.kDriveMotorGearRatio / Constants.ModuleConstants.kWheelDiameterMeters;
          
        // Converts rpm to encoder units per 100 milliseconds
        double desiredDriveEncoderUnitsPer100MS = desiredDriveRPM / 600.0 * 2048;
    
        // Sets the drive motor's speed using the built in pid controller
        driveMotor.set(desiredDriveEncoderUnitsPer100MS);
        SmartDashboard.putNumber("Drive Motor Alternative", desiredDriveEncoderUnitsPer100MS);
    
        // Calculate the turning motor output from the turn PID controller.
        double turnOutput =
        azimuthPIDController.calculate(turnRadians, optimizedDesiredState.angle.getRadians())
            + azimuthFeedForward.calculate(azimuthPIDController.getSetpoint().velocity);
        SmartDashboard.putNumber("Turnign Motor Alternative", turnOutput / 12);

        turningMotor.set(turnOutput / 12);
    }


























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