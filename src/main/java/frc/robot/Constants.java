package frc.robot;


import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {

    public static final class ModuleConstants {

        public static final double kEncoderCPR = 2048;

        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 1 / 8.14; //L1
        public static final double kTurningMotorGearRatio = 1 / (150/7);
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = 0.5;
    }

    public static final class DriveConstants {

        public static final double kTrackWidth = 0.5334; //meters //Units.inchesToMeters(21);
        // Distance between right and left wheels
        public static final double kWheelBase = 0.6477;//meters //Units.inchesToMeters(25.5);
        // Distance between front and back wheels

        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2));
        
        public static final int kFrontLeftDriveMotorPort = 1;
        public static final int kFrontLeftTurningMotorPort = 2;
        public static final int kFrontLeftDriveAbsoluteEncoderPort = 12;

        public static final int kFrontRightDriveMotorPort = 3;
        public static final int kFrontRightTurningMotorPort = 4;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 11;

        public static final int kBackLeftDriveMotorPort = 5;
        public static final int kBackLeftTurningMotorPort = 6;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 10;

        public static final int kBackRightDriveMotorPort = 7;
        public static final int kBackRightTurningMotorPort = 8;
        public static final int kBackRightDriveAbsoluteEncoderPort = 9;

        public static final boolean kFrontLeftTurningEncoderReversed = false;
        public static final boolean kBackLeftTurningEncoderReversed = false;
        public static final boolean kFrontRightTurningEncoderReversed = false;
        public static final boolean kBackRightTurningEncoderReversed = false;

        public static final boolean kFrontLeftDriveEncoderReversed = true;
        public static final boolean kBackLeftDriveEncoderReversed = false;
        public static final boolean kFrontRightDriveEncoderReversed = true;
        public static final boolean kBackRightDriveEncoderReversed = false;


        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = true;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = true;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = true;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = true;

        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = ((Math.PI/6) - 0.122173 - 0.05+0.018408-0.056734+0.006136-0.204019+.193281); //offset in radians #12
        // 12

        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = (-6.280105+Math.PI/4+.67 - .02 - 0.001534+0.019942-0.076707+0.0730732-.104311); //offset in radians #10
        //10
        
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = (Math.PI+0.046019-0.036937+1.040037+Math.PI+0.058291-.122297-.052156); //offset in radians #11
        // 11
        
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = (0); //offset in radians #9
        // 9

        public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond ; //change denomenator
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 4; //change denomenator
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;

        public static class AutoConstants {
            private AutoConstants() {
                throw new IllegalStateException("Utility Class");
            }
    
            //Trajectory following values
            public static final double MAX_VELOCITY_PERCENT_OUTPUT = kPhysicalMaxSpeedMetersPerSecond;
;
            public static final double MAX_ACCELERATION_PERCENT_OUTPUT = 1.5;
    
            public static final PIDController THETA_CONTROLLER =
                    new PIDController(2.0, 0.0, 0.0);
    
            public static final PIDController CONTROLLER_X =
                new PIDController(2, 0, 0);
            public static final PIDController CONTROLLER_Y =
                new PIDController(2, 0, 0);
    
            public static final PIDConstants CONSTANTS_X =
                    new PIDConstants(2.0, 0, 0);
    
            public static final PIDConstants THETA_CONSTANTS =
                    new PIDConstants(2.0, 0.0, 0.0);
            
            //Auto balance constants
            public static final double BALANCE_P = -0.04;
            public static final double DESIRED_BALANCE_ANGLE = 0;
            public static final double ACCEPTABLE_BALANCE_ANGLE = 1.5;
            public static final double BALANCE_D = 0.1;
    
            public static final Transform2d CENTER_TRANSLATION = new Transform2d(
                    new Translation2d(0.75, 0.0),
                    new Rotation2d(0.0)
            );
    
            public static final Transform2d LEFT_TRANSLATION = new Transform2d(
                    new Translation2d(0.75, 0.55),
                    new Rotation2d()
            );
    
            public static final Transform2d HUMAN_PLAYER_RIGHT_TRANSLATION = new Transform2d(
                    new Translation2d(0.6, 0.6),
                    new Rotation2d()
            );
    
            public static final Transform2d HUMAN_PLAYER_LEFT_TRANSLATION = new Transform2d(
                    new Translation2d(0.6, -0.6),
                    new Rotation2d()
            );
    
            public static final Transform2d RIGHT_TRANSLATION = new Transform2d(
                    new Translation2d(0.75, -0.6),
                    new Rotation2d()
            );
            public static final double kMaxSpeedMetersPerSecond = 0.5;
            public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI*0.5;
            public static final double kMaxAccelerationMetersPerSecondSquared = 0.25;
            public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI/4;
            public static final double kPXController = 0.75; //multiplier for controller PID control
            public static final double kPYController = 0.75; //multiplier for controller PID control
            public static final double kPThetaController = 1.75; //multiplier for controller PID control
    
            public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                    new TrapezoidProfile.Constraints(
                            kMaxAngularSpeedRadiansPerSecond,
                            kMaxAngularAccelerationRadiansPerSecondSquared);
        }
    
        public static final class OIConstants {
            public static final int kDriverControllerPort = 0;
            public static final int kOperatorControllerPort = 1;
    
            public static final int kDriverYAxis = 1;
            public static final int kDriverXAxis = 0;
            public static final int kDriverRotAxis = 4;
            public static final int kDriverFieldOrientedButtonIdx = 6;
            public static final int kAlignWithTargetButton = 5;
            public static final int kResetDirectionButton = 4;
    
            public static final int kRotate0Button = 3;
            public static final int kRotate180Button = 2;
            public static final int kExtendFullButton = 4;
            public static final int kRetractButton = 1;
            public static final int kToggleGrabButton = 10;
            public static final int kReverseGrabButton = 6;
            public static final int kForwardGrabButton = 5;
            public static final int kManuelButton = 7;
    
            public static final double kDeadband = 0.05;
        }
    
        public enum ModulePosition {
            FRONT_LEFT,
            FRONT_RIGHT,
            BACK_LEFT,
            BACK_RIGHT
        }
        public static final int LimitSwitchDIO = 1;
        public static final int WheeliePWM = 0;
        }

    public static final class MechanismConstants {
        public static final int kTelescopePort = 20;
        public static final int kPivotPort = 21;
        public static final int kPivotFollowPort = 22;

        public static final int kTelescopeEncoderCPR = 2048; //count per revolution
        public static final int kPivotEncoderCPR = 42; //counts per revolution

        public static final double kReductionTelescope = 35;
        public static final double kRotationsToFullExtentTelescope = 8;
        public static final double kReductionPivot = 100*(56/18);

        public static final double kPTelescope = 1;
        public static final double kITelescope = 0;
        public static final double kDTelescope = 0;
        public static final double kFTelescope = 0;
        public static final double kMinTelescope = -1;
        public static final double kMaxTelescope = 1;
        public static final int kTimeoutMsTelescope = 30;
        public static final int kPIDLoopIdxTelescope = 0;
        public static final int kSlotIdxTelescope = 0;

        public static final double kPPivot = 1;
        public static final double kIPivot = 0;
        public static final double kDPivot = 0;
        public static final double kMinPivot = -.2;
        public static final double kMaxPivot = .2;
    }


}