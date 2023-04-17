package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoCommands.ArmAutoMovement;
import frc.robot.commands.AutoCommands.PistonIntakeMovement;
import frc.robot.commands.AutoCommands.PistonIntakeStatus;
import frc.robot.commands.AutoCommands.TransferGrip;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.linearSlideArm;
import frc.robot.subsystems.outtakeTransfer;
import frc.robot.subsystems.pistonIntake;

public class HighCubeDrop extends SequentialCommandGroup {

  public HighCubeDrop
  (
    linearSlideArm arm,
    outtakeTransfer outtakeTransferSubsystem,
    pistonIntake pistonIntakeSubsystem,
    SwerveSubsystem swerveSubsystem
  ) 
  {

    addCommands
    (
      // new TransferGrip(outtakeTransferSubsystem, "open"),
      // new PistonIntakeStatus(pistonIntakeSubsystem, "close"),
      // new PistonIntakeMovement(pistonIntakeSubsystem, "down"),
      // new ArmAutoMovement(arm, "high"),
      // new TransferGrip(outtakeTransferSubsystem, "close"),
      // new ArmAutoMovement(arm, "reset"),
      // new InstantCommand(() -> swerveSubsystem.stopModules())
     // new PistonIntakeMovement(pistonIntakeSubsystem, "off"),
      new TransferGrip(outtakeTransferSubsystem, "timedOpen").alongWith(new WaitCommand(0.5)),


      //new PistonIntakeMovement(pistonIntakeSubsystem, "off"),
      //new PistonIntakeMovement(pistonIntakeSubsystem, "downTimed"),

      new HighCubeUpParrallelDeadline(arm, outtakeTransferSubsystem, pistonIntakeSubsystem, swerveSubsystem)
      ,new TransferGrip(outtakeTransferSubsystem, "close").alongWith(new WaitCommand(0.1))
      ,new ArmAutoMovement(arm, "reset")


      
    );
  }
}
