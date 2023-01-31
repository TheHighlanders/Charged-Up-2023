// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class vision extends SubsystemBase {
  /** Creates a new vision. */
  private NetworkTable visionNT = NetworkTableInstance.getDefault().getTable("limelight");
  private NetworkTableEntry tv = visionNT.getEntry("tv");
  private NetworkTableEntry tx = visionNT.getEntry("tx");
  private NetworkTableEntry ty = visionNT.getEntry("ty");
  private NetworkTableEntry pipe = visionNT.getEntry("pipeline");
  private NetworkTableEntry tid = visionNT.getEntry("tid");

  public double tvVal;
  public double txVal;
  public double tyVal;
  public double pipeVal;
  public double tidVal;

  public vision() {

  }

  @Override
  public void periodic() {
    updateNetworkTables();

    SmartDashboard.putNumber("targetX", txVal);
    SmartDashboard.putNumber("targetY", tyVal);
    SmartDashboard.putNumber("tag ID", tidVal);

    // This method will be called once per scheduler run
  }

  public void updateNetworkTables() {

    DriverStation.reportWarning("Entry " + tx, false);
    tvVal = tv.getDouble(0.0);
    txVal = tx.getDouble(0.0);
    tyVal = ty.getDouble(0.0);
    tidVal = tid.getDouble(0.0);
    pipeVal = pipe.getDouble(0.0);

  }
}
