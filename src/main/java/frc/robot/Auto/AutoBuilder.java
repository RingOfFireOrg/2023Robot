package frc.robot.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

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

    private void auto1() {
        startPath = new AutoPath(robotContainer.swerveSubsystem, "");

        autoCommand.addCommands(
            new ParallelCommandGroup(
                //startPath.zeroHeading(),
                startPath.setBrake(),
                startPath.odometryReset(),
                startPath.getAutoPath()
            )
        );
    }
    private void auto2() {
        startPath = new AutoPath(robotContainer.swerveSubsystem, "AUTO2");

        autoCommand.addCommands(
            new ParallelCommandGroup(
                //startPath.zeroHeading(),
                startPath.setBrake(),
                startPath.odometryReset(),
                startPath.getAutoPath()
            )
        );
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


}