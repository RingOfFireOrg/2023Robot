package frc.robot.Auto;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Robot.AutoModes;
import frc.robot.commands.CommandGroups.HighCubeDrop;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.linearSlideArm;
import frc.robot.subsystems.outtakeTransfer;
import frc.robot.subsystems.pistonIntake;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;

public class AutoBuilder {
    private RobotContainer robotContainer;
    private AutoModes autoMode;
    private SequentialCommandGroup autoCommand;
    private AutoPath startPath;

    public Command build() {
        autoCommand = new SequentialCommandGroup();

        switch (autoMode) {
            case AUTO1:
                auto11();
                break;
            case AUTO2:
                auto2();
                break;
            case AUTO3:
                auto3();
                break;
            case AUTO4:
                auto4();
                break;
            case AUTO5:
                auto5();
                break;
            case AUTO6:
                auto6();
                break;
        }

        //autoCommand.beforeStarting(startPath.odometryReset());
        return autoCommand;
    }

    public void setRobotContainer(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
    }

    public void setAutoMode(AutoModes autoMode) {
        this.autoMode = autoMode;
    }

    public void auto11() {
        SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
        linearSlideArm armSubsystem = new linearSlideArm();
        pistonIntake pistonIntakeSubsystem = new pistonIntake();
        outtakeTransfer outtakeTransferSubsystem = new outtakeTransfer();

        startPath = new AutoPath(robotContainer.swerveSubsystem, "[Both][Middle]ScoreLeaveBalence");

        autoCommand.addCommands(
            new HighCubeDrop(armSubsystem, outtakeTransferSubsystem, pistonIntakeSubsystem, swerveSubsystem),
            new ParallelCommandGroup(
                //startPath.zeroHeading(),
                startPath.setBrake(),
                startPath.odometryReset(),
                startPath.getAutoPath()
            ),
            new PIDAutoBalancer(swerveSubsystem)
        );
    }
    private void auto2() {
        // SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
        // linearSlideArm armSubsystem = new linearSlideArm();
        // pistonIntake pistonIntakeSubsystem = new pistonIntake();
        // outtakeTransfer outtakeTransferSubsystem = new outtakeTransfer();

        // //This will load the file "FullAuto.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
        // //for every path in the group
        // PathPlannerTrajectory examplePath = PathPlanner.loadPath("[Both][Middle]ScoreLeaveBalence", new PathConstraints(4, 3));

        // //This is just an example event map. It would be better to have a constant, global event map
        // // in your code that will be used by all path following commands.
        // HashMap<String, Command> eventMap = new HashMap<>();
        // eventMap.put("ScoreHigh", new HighCubeDrop(armSubsystem, outtakeTransferSubsystem, pistonIntakeSubsystem, swerveSubsystem));
        // eventMap.put("PIDBalence", new PIDAutoBalancer(swerveSubsystem));


        

        // SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        //     trajectory,
        //     swerveSubsystem::getPose,
        //     DriveConstants.kDriveKinematics,
        //     xController,
        //     yController,
        //     thetaController,
        //     swerveSubsystem::setModuleStates,
        //     swerveSubsystem);

        // Command fullAuto = autoBuilder.fullAuto(pathGroup);
    }
    private void auto3() {
        startPath = new AutoPath(robotContainer.swerveSubsystem, "AUTO3");

        autoCommand.addCommands(
            new ParallelCommandGroup(
                //startPath.zeroHeading(),
                startPath.setBrake(),
                startPath.odometryReset(),
                startPath.getAutoPath()
            )
        );
    }
    private void auto4() {
        startPath = new AutoPath(robotContainer.swerveSubsystem, "AUTO4");

        autoCommand.addCommands(
            new ParallelCommandGroup(
                //startPath.zeroHeading(),
                startPath.setBrake(),
                startPath.odometryReset(),
                startPath.getAutoPath()
            )
        );
    }
    private void auto5() {
        startPath = new AutoPath(robotContainer.swerveSubsystem, "AUTO5");

        autoCommand.addCommands(
            new ParallelCommandGroup(
                //startPath.zeroHeading(),
                startPath.setBrake(),
                startPath.odometryReset(),
                startPath.getAutoPath()
            )
        );
    }
    private void auto6() {
        startPath = new AutoPath(robotContainer.swerveSubsystem, "AUTO6");

        autoCommand.addCommands(
            new ParallelCommandGroup(
                //startPath.zeroHeading(),
                startPath.setBrake(),
                startPath.odometryReset(),
                startPath.getAutoPath()
            )
        );
    }




    // public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
    //     SwerveSubsystem swervey = new SwerveSubsystem();

    //     PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
    //     PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
    //     ProfiledPIDController thetaController = new ProfiledPIDController(
    //         AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);       
         
    //     return new SequentialCommandGroup(
    //          new InstantCommand(() -> {
    //            // Reset odometry for the first path you run during auto
    //            if(isFirstPath){
    //                this.resetOdometry(traj.getInitialHolonomicPose());
    //            }
    //          }),

    //          new PPSwerveControllerCommand(
    //              traj, 
    //              swervey::getPose, // Pose supplier
    //              /*this.DriveConstants.kDriveKinematics,*/ // SwerveDriveKinematics
    //              xController, // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
    //              yController, // Y controller (usually the same values as X controller)
    //              thetaController, // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
    //              this::setModuleStates, // Module states consumer
    //              true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
    //              this,
    //              null,
    //              null // Requires this drive subsystem
    //          )
    //      );
    //  }
     
}