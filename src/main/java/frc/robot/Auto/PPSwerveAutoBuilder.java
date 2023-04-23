package frc.robot.Auto;

import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.CommandGroups.HighCubeDrop;
import frc.robot.subsystems.linearSlideArm;
import frc.robot.subsystems.outtakeTransfer;
import frc.robot.subsystems.pistonIntake;

import frc.robot.subsystems.SwerveSubsystem;

import java.util.ArrayList;
import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class PPSwerveAutoBuilder extends SequentialCommandGroup {
  public PPSwerveAutoBuilder
  (
    SwerveSubsystem driveSubsystem,
    linearSlideArm arm,
    outtakeTransfer wheelie,
    pistonIntake piston,
    String autoPath,
    double maxVel,
    double maxAccel
  )

{

// This will load the file "FullAuto.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
// for every path in the group

ArrayList<PathPlannerTrajectory> pathGroup = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("ihopethisworkslol",new PathConstraints(1, 1));

// This is just an example event map. It would be better to have a constant, global event map
// in your code that will be used by all path following commands.
HashMap<String, Command> eventMap = new HashMap<>();
eventMap.put("ScoreHigh", new HighCubeDrop(arm, wheelie, piston, driveSubsystem));
eventMap.put("Balence", new ReversePIDAutoBalancer(driveSubsystem));
eventMap.put("ScoreHighhh", new InstantCommand());

// Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
    driveSubsystem::getPose, // Pose2d supplier
    driveSubsystem::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
    DriveConstants.kDriveKinematics,
    new PIDConstants(0.6, 0.0, 0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
    new PIDConstants(1.75, 0.0, 0), // PID constants to correct for rotation error (used to create the rotation controller)
    driveSubsystem::setModuleStates, // Module states consumer used to output to the drive subsystem
    eventMap,
    true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
    driveSubsystem // The drive subsystem. Used to properly set the requirements of path following commands
);

    autoBuilder.fullAuto(pathGroup).schedule();
    }
}