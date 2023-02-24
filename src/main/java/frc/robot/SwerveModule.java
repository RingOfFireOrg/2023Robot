/* Big thanks to Team 364 for the base code. */

package frc.robot;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.util.ModuleStateOptimizer;
import frc.lib.util.SwerveModuleConstants;

public class SwerveModule {
    public final int moduleNumber;
    private final Rotation2d angleOffset;
    private Rotation2d lastAngle = Rotation2d.fromDegrees(0);

    private final CANSparkMax angleMotor;
    private final CANSparkMax driveMotor;
    private final CANCoder absoluteEncoder;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder angleEncoder;

    private final SparkMaxPIDController drivePIDController;
    private final SparkMaxPIDController anglePIDController;

    private final SimpleMotorFeedforward feedforward =
            new SimpleMotorFeedforward(
                    Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    /* Sim Caches (basically im lazy and don't want to use the rev physics sim) */
    private double simSpeedCache;
    private Rotation2d simAngleCache = Rotation2d.fromDegrees(0);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;

        /* Absolute Encoder */
        absoluteEncoder = new CANCoder(moduleConstants.cancoderID);
        configAngleEncoder();

        /* Angle Motor */
        angleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        angleEncoder = angleMotor.getEncoder();
        anglePIDController = angleMotor.getPIDController();
        configAngleMotor();

        /* Drive motor */
        driveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
        driveEncoder = driveMotor.getEncoder();
        drivePIDController = driveMotor.getPIDController();
        configDriveMotor();

        lastAngle = getState().angle;
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState =
                ModuleStateOptimizer.optimize(
                        desiredState,
                        getState().angle); // Custom optimize command, since default WPILib optimize
        // assumes continuous controller which CTRE is not

        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
        simSpeedCache = desiredState.speedMetersPerSecond;
        simAngleCache = desiredState.angle;
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            // mDriveMotor_ctre.set(ControlMode.PercentOutput, percentOutput);
            driveMotor.set(percentOutput);
        } else {
            driveMotor
                    .getPIDController()
                    .setReference(
                            desiredState.speedMetersPerSecond,
                            ControlType.kVelocity,
                            0,
                            feedforward.calculate(desiredState.speedMetersPerSecond),
                            SparkMaxPIDController.ArbFFUnits.kVoltage);
            // mDriveMotor.set(ControlMode.Velocity, desiredState.speedMetersPerSecond,
            // DemandType.ArbitraryFeedForward,
            // feedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    private void setAngle(SwerveModuleState desiredState) {
        Rotation2d angle =
                (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01))
                        ? lastAngle
                        : desiredState.angle; // Prevent rotating module if speed is less than 1%.
        // Prevents Jittering.
        // mAngleMotor_ctre.set(ControlMode.Position,
        // Conversions.degreesToFalcon(angle.getDegrees(),
        // Constants.DriveSubsystem.angleGearRatio));
        angleMotor.getPIDController().setReference(angle.getDegrees(), ControlType.kPosition);
        lastAngle = angle;
    }

    private Rotation2d getAngle() {
        if (Robot.isReal()) return Rotation2d.fromDegrees(angleEncoder.getPosition());
        return simAngleCache; // If sim.
    }

    public Rotation2d getAbsoluteAngle() {
        return Rotation2d.fromDegrees(absoluteEncoder.getAbsolutePosition());
    }

    public void resetToAbsolute() {
        angleEncoder.setPosition(getAbsoluteAngle().getDegrees() - angleOffset.getDegrees());
    }

    private void configAngleEncoder() {
        absoluteEncoder.configFactoryDefault();
        absoluteEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
    }

    private void configAngleMotor() {
        angleMotor.restoreFactoryDefaults();
        angleMotor.setSmartCurrentLimit(Constants.Swerve.ANGLE_SMART_CURRENT_LIMIT);
        angleMotor.setSecondaryCurrentLimit(Constants.Swerve.ANGLE_SECONDARY_CURRENT_LIMIT);
        angleMotor.setInverted(Constants.Swerve.ANGLE_MOTOR_INVERT);
        //angleMotor.setIdleMode(Constants.Swerve.ANGLE_NEUTRAL_MODE);
        angleMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);

        angleEncoder.setPositionConversionFactor(
                (1
                                / Constants.Swerve.chosenModule
                                        .angleGearRatio) // We do 1 over the gear ratio because 1
                        // rotation of the motor is < 1 rotation of
                        // the module
                        * 360); // 1/360 rotations is 1 degree, 1 rotation is 360 degrees.
        resetToAbsolute();

        anglePIDController.setP(Constants.Swerve.angleKP);
        anglePIDController.setI(Constants.Swerve.angleKI);
        anglePIDController.setD(Constants.Swerve.angleKD);
        anglePIDController.setFF(Constants.Swerve.angleKF);

        // TODO: Adjust this latter after we know the pid loop is not crazy
        angleMotor.getPIDController().setOutputRange(-.25, .25);
    }

    private void configDriveMotor() {
        driveMotor.restoreFactoryDefaults();
        driveMotor.setSmartCurrentLimit(Constants.Swerve.DRIVE_SMART_CURRENT_LIMIT);
        driveMotor.setSecondaryCurrentLimit(Constants.Swerve.DRIVE_SECONDARY_CURRENT_LIMIT);
        driveMotor.setInverted(Constants.Swerve.DRIVE_MOTOR_INVERT);
        // driveMotor.setIdleMode(Constants.Swerve.DRIVE_NEUTRAL_MODE);
        driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        driveMotor.setOpenLoopRampRate(Constants.Swerve.OPEN_LOOP_RAMP);
        driveMotor.setClosedLoopRampRate(Constants.Swerve.CLOSED_LOOP_RAMP);

        driveEncoder.setVelocityConversionFactor(
                1 / Constants.Swerve.DRIVE_GEAR_RATIO // 1/gear ratio because the wheel spins slower than
                        // the motor.
                        * Constants.Swerve.WHEEL_CIRCUMFERENCE // Multiply by the circumference to get meters
                        // per minute
                        / 60); // Divide by 60 to get meters per second.
        driveEncoder.setPosition(0);

        drivePIDController.setP(Constants.Swerve.driveKP);
        drivePIDController.setI(Constants.Swerve.driveKI);
        drivePIDController.setD(Constants.Swerve.driveKD);
        drivePIDController.setFF(
                Constants.Swerve
                        .driveKF); // Not actually used because we specify our feedforward when we
        // set our speed.

        // TODO: Remove after we know the pid loop isn't wild
        drivePIDController.setOutputRange(-.5, .5);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                Robot.isReal() ? driveEncoder.getVelocity() : simSpeedCache, getAngle());
    }

    public SwerveModulePosition getPosition() {
        //SwerveModulePosition modPos = new SwerveModulePosition();
       // double position = Conversions.falconToMeters(driveMotor.getSelectedSensorPosition(),
        //Constants.Swerve.DRIVE_GEAR_RATIO, Constants.Swerve.WHEEL_CIRCUMFERENCE);

    //Rotation2d angle = Rotation2d.fromDegrees(Conversions.falconToDegrees(
        //angleMotor.getSelectedSensorPosition(), Constants.Swerve.ANGLE_GEAR_RATIO));
    //return new SwerveModulePosition(position, angle);

    return new SwerveModulePosition(
        1.0,
        getAngle());
   
    }

    public Rotation2d getCanCoder() {
        //TODO: Actually code this properly
        return new Rotation2d(1,1);
    }
}

