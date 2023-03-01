// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight2 extends SubsystemBase {
  /** Creates a new Limelight2. */
  public Limelight2() {
    // Need to tune these values
    final double visionrange = 0; 
    final double lowScoringHeight = 0; 
    final double middleScoringHeight = 0;
    final double highScoringHeight = 0;
  } 
  public double[] getVisionVals() {
    // https://docs.limelightvision.io/en/latest/networktables_api.html
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    // Horizontal Offset From Crosshair To Target (-29.8 to 29.8deg)
    double x = table.getEntry("tx").getDouble(0.0);

    // Vertical Offset From Crosshair To Target (-24.85 to 24.85deg)
    double y = table.getEntry("ty").getDouble(0.0);

    // Valid target in vision (0 or 1)
    double v = table.getEntry("ty").getDouble(0.0);

    // post to smart dashboard
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);

    double[] arr = { x, y, v };
    return arr;
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
