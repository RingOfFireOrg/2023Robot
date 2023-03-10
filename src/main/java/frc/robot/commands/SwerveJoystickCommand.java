package frc.robot.commands;

import java.util.function.Supplier;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystickCommand extends CommandBase {

    private final SwerveSubsystem swerveSubsystem;

    private final Supplier<Double> xSpdFunctionField, ySpdFunctionField, xSpdFunctionRobot, ySpdFunctionRobot, turningSpdFunctionLeft, turningSpdFunctionRight;
    private final Supplier<Boolean> fieldOrientedFunction, alignFunction, resetDirection;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

    public final double cameraHeight = Units.inchesToMeters(5);// replace number with height of camera on robot
    public final double targetHeight = Units.feetToMeters(5);// replace number with height of targets
    public final double cameraPitch = Units.degreesToRadians(65);// replace number with angle of camera

    PhotonCamera camera = new PhotonCamera("gloworm");
    private final XboxController driveController = new XboxController(0);

    //pid constants
    final double linearP = 0.0;
    final double linearD = 0.0;

    final double angularP = 0.1;
    final double angularD = 0.005;
    PIDController turnController = new PIDController(angularP, 0, angularD);
    boolean manuelMode = false;
    boolean fieldOrientTrue = true;
    

    public SwerveJoystickCommand(SwerveSubsystem swerveSubsystem, 
            Supplier<Double> xSpdFunctionField, 
            Supplier<Double> ySpdFunctionField, 

            Supplier<Double> xSpdFunctionRobot,
            Supplier<Double> ySpdFunctionRobot,

            Supplier<Double> turningSpdFunctionLeft,
            Supplier<Double> turningSpdFunctionRight,
            Supplier<Boolean> fieldOrientedFunction, 

            Supplier<Boolean> alignButton, 
            Supplier<Boolean> resetDirectionButton) {

        this.swerveSubsystem = swerveSubsystem;

        this.xSpdFunctionField = xSpdFunctionField;
        this.ySpdFunctionField = ySpdFunctionField;

        this.xSpdFunctionRobot = xSpdFunctionRobot;
        this.ySpdFunctionRobot = ySpdFunctionRobot;

        this.turningSpdFunctionLeft = turningSpdFunctionLeft;
        this.turningSpdFunctionRight = turningSpdFunctionRight;

        this.fieldOrientedFunction = fieldOrientedFunction;
        this.alignFunction = alignButton;
        this.resetDirection = resetDirectionButton;

        // this.rotate0 = rotate0Button;
        // this.rotate180 = rotate180Button;
        // this.extendFull = extendFullButton;
        // this.retract = retractButton;
        // this.toggleGrab = toggleGrabButton;
        // this.reverseGrab = reverseGrabButton;
        // this.forwardGrab = forwardGrabButton;

        // this.manuel = manuelButton;

        //this.manuelPivotPOV = manuelPivotPOV;
        //this.manuelTelescopePOV = manuelPivotPOV;

        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        addRequirements(swerveSubsystem);

    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {


        if (xSpdFunctionField.get() >= 0.1 || xSpdFunctionField.get() <= -0.1 || ySpdFunctionField.get() >= 0.1 || ySpdFunctionField.get() <= -0.1) 
        {
            // 1. Get real-time joystick inputs
            double xSpeed = xSpdFunctionField.get();
            double ySpeed = ySpdFunctionField.get();
            double turningSpeed = turningSpdFunctionLeft.get() - turningSpdFunctionRight.get();

            // 2. Apply deadband
            xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
            ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
            turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

            // 3. Make the driving smoother
            xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
            ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
            turningSpeed = turningLimiter.calculate(turningSpeed)
                    * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

            // 4. Construct desired chassis speeds
            ChassisSpeeds chassisSpeeds;
            SmartDashboard.putNumber("Rotation 2d Number", swerveSubsystem.getRotation2dButaDouble());
            
            
            // chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds (
            //     xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());

            if (!alignFunction.get()) {
                // Relative to field
                chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
            } 
            else {
                // Relative to robot
                chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
            }

            // 5. Convert chassis speeds to individual module states
            SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

            // 6. Output each module states to wheels
            swerveSubsystem.setModuleStates(moduleStates);
        }
        else 
        {
            // 1. Get real-time joystick inputs
            double xSpeed = xSpdFunctionRobot.get();
            double ySpeed = ySpdFunctionRobot.get();

            double turningSpeed = turningSpdFunctionLeft.get() - turningSpdFunctionRight.get();

            // 2. Apply deadband
            xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
            ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
            turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

            // 3. Make the driving smoother
            xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
            ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
            turningSpeed = turningLimiter.calculate(turningSpeed)
                    * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

            // 4. Construct desired chassis speeds
            ChassisSpeeds chassisSpeeds;
            SmartDashboard.putNumber("Rotation 2d Number", swerveSubsystem.getRotation2dButaDouble());

            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);

            // 5. Convert chassis speeds to individual module states
            SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

            // 6. Output each module states to wheels
            swerveSubsystem.setModuleStates(moduleStates);
        }
        




















    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}