package frc.robot.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Robot.AutoModes;
import frc.robot.RobotContainer;

public class AutoBuilder {
    private RobotContainer robotContainer;
    private AutoModes autoMode;
    private SequentialCommandGroup autoCommand;
    private AutoPath startPath;

    public Command build() {
        autoCommand = new SequentialCommandGroup();

        switch (autoMode) {
            case AUTO1:
                auto1();
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

    private void auto1() {
        startPath = new AutoPath(robotContainer.swerveSubsystem, "AUTO1");

        autoCommand.addCommands(
            new ParallelCommandGroup(
                //startPath.zeroHeading(),
                startPath.setBrake(),
                startPath.odometryReset(),
                startPath.getAutoPath()
            )
        );
    }

}