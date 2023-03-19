package frc.robot.commands;
import java.util.Map;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;


public class AutoUtils {

    public enum ScoringHeights {
        HIGH,
        MIDDLE,
        LOW
    }

    public enum StartingZones {
        LEFT,
        MIDDLE,
        RIGHT
    }

    private AutoUtils() {
        throw new IllegalStateException("Utility Class");
    }

    // Default Constants
    private static final PathConstraints m_defaultConfig = new PathConstraints(
        Constants.DriveConstants.kTeleDriveMaxSpeedMetersPerSecond, Constants.DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        

//    private static final PathPlannerTrajectory m_defaultAutoGen = PathPlanner.loadPath("DefaultPath", m_defaultConfig);

//    private static final SwerveAutoBuilder defaultAutoFactory = new SwerveAutoBuilder(
//
//    )

    public static PathConstraints getDefaultConstraints() {
        return m_defaultConfig;
    }

    public static Command getAutoRoutine(PathPlannerTrajectory traj, SwerveSubsystem swerve, boolean firstTrajectory){
        return new SequentialCommandGroup(
            new ConditionalCommand(new InstantCommand(() ->
                    swerve.resetPose(
                    PathPlannerTrajectory.transformTrajectoryForAlliance(traj, DriverStation.getAlliance()).getInitialHolonomicPose())),
                    new InstantCommand(),
                    () -> firstTrajectory
            ),


            new PPSwerveControllerCommand(traj,
            swerve::getPose, 
            frc.robot.Constants.DriveConstants.kDriveKinematics, 
            frc.robot.Constants.DriveConstants.AutoConstants.CONTROLLER_X, 
            frc.robot.Constants.DriveConstants.AutoConstants.CONTROLLER_Y, 
            frc.robot.Constants.DriveConstants.AutoConstants.THETA_CONTROLLER,
            swerve::setModuleStates,
            true,
            swerve)
        );
    }
    
    public static Command getAutoEventRoutine(PathPlannerTrajectory traj, Map<String, Command> events, SwerveSubsystem swerve) {
        return new FollowPathWithEvents(getAutoRoutine(traj, swerve, true), traj.getMarkers(), events);
    }
}
