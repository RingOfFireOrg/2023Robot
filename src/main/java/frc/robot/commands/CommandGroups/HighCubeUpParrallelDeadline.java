package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.commands.AutoCommands.ArmAutoMovement;
import frc.robot.commands.AutoCommands.PistonIntakeMovement;
import frc.robot.commands.AutoCommands.TransferSetGrip;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.linearSlideArm;
import frc.robot.subsystems.outtakeTransfer;
import frc.robot.subsystems.pistonIntake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HighCubeUpParrallelDeadline extends ParallelDeadlineGroup {
  public HighCubeUpParrallelDeadline
  (
    linearSlideArm arm,
    outtakeTransfer outtakeTransferSubsystem,
    pistonIntake pistonIntakeSubsystem,
    SwerveSubsystem swerveSubsystem
  ) {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    super(new ArmAutoMovement(arm, "high"));

    //addCommands(new PistonIntakeMovement(pistonIntakeSubsystem, "downTimed"));
    addCommands(new TransferSetGrip(outtakeTransferSubsystem, 0.4));
    ;
  }
}
