package frc.robot.Auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.pistonIntake;

public class pistonOpenClose extends CommandBase {
  pistonIntake piston;
  String pistonState;
  public pistonOpenClose(pistonIntake piston, String pistonState) {
    this.piston = piston;
    this.pistonState = pistonState;
    addRequirements(piston);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (pistonState == "open") {
      piston.openPiston();
    }
    else if (pistonState == "close") {
      piston.closePiston();

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
