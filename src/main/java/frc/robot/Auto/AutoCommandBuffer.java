package frc.robot.Auto;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutoCommandBuffer extends CommandBase {
  public AutoCommandBuffer() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    new WaitCommand(.8);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
