package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.HashMap;

public class MobilityCommandGroup extends SequentialCommandGroup {
    public MobilityCommandGroup(SwerveSubsystem m_swerve, frc.robot.commands.AutoUtils.StartingZones m_start) {
        // Get proper path for driver POV
        DriverStation.Alliance alliance = DriverStation.getAlliance();
        PathPlannerTrajectory trajectory;
        PathConstraints constraints = AutoUtils.getDefaultConstraints();

        switch (m_start) {
            case LEFT:
                if (alliance == DriverStation.Alliance.Blue) {
                    trajectory = PathPlanner.loadPath("Mobility Left", constraints);
                } else {
                    trajectory = PathPlanner.loadPath("Mobility Right", constraints);
                }
                break;
            case RIGHT:
                if (alliance == DriverStation.Alliance.Blue) {
                    trajectory = PathPlanner.loadPath("Mobility Right", constraints);
                } else {
                    trajectory = PathPlanner.loadPath("Mobility Left", constraints);
                }
                break;
            case MIDDLE:
                trajectory = PathPlanner.loadPath("Mobility Middle", constraints);
                break;
            default:
                trajectory = new PathPlannerTrajectory();
        }

        addCommands(m_swerve.getAutoBuilder(new HashMap<>()).fullAuto(trajectory));
    }
}