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
import frc.robot.RobotContainer;

public class vision extends SubsystemBase {
  /** Creates a new vision. */
  private NetworkTable visionNT = NetworkTableInstance.getDefault().getTable("limelight");
  private NetworkTableEntry tv = visionNT.getEntry("tv");
  private NetworkTableEntry tx = visionNT.getEntry("tx");
  private NetworkTableEntry ty = visionNT.getEntry("ty");
  private NetworkTableEntry tPoseRobotSpace = visionNT.getEntry("targetpose_robotspace");
  private NetworkTableEntry pipe = visionNT.getEntry("pipeline");
  private NetworkTableEntry tid = visionNT.getEntry("tid");

  public double tvVal;
  public double txVal;
  public double tyVal;
  public double pipeVal;
  public double tidVal;

  private double robotX;
  private double robotY;
  private double theta;
  private double phi;

  private double distanceToTarget;

  private double odometerOffsetX;
  private double odometerOffsetY;

  public double tagOdoX;
  public double tagOdoY;
  private double tagOffsetX;
  private double tagOffsetY;

  private double[] poseArray = new double[6];

  public double Xr;
  public double Yr; //Robot Space position of tag

  public double Z; //Hypotenuse of All triangle

  public double alpha; //Angle of corner of Robot Space triangle at robot

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
    updateZalpha();
    calulateTag();
    DriverStation.reportWarning("Entry " + tx, false);
    tvVal = tv.getDouble(0.0);
    txVal = tx.getDouble(0.0);
    tyVal = ty.getDouble(0.0);
    tidVal = tid.getDouble(0.0);
    pipeVal = pipe.getDouble(0.0);

    poseArray = NetworkTableInstance.getDefault().getTable("limelight").getEntry("targetpose_robotspace")
        .getDoubleArray(new double[6]);
    Xr = poseArray[0];
    Yr = poseArray[2]; //0 & 1 are guesses, should be the index of the transform in X and Y

    SmartDashboard.putNumber("TargetPoseX in Robot Space", Xr);
    SmartDashboard.putNumber("TargetPoseY in Robot Space", Yr);

  }

  public void updateZalpha() {
    if (tvVal == 1) {
      Z = Math.hypot(Xr, Yr);
      SmartDashboard.putNumber("Z", Z);
        alpha = Math.atan2(Yr, Xr);
      
    }

  }

  public void calulateTag(){
    distanceToTarget = Math.hypot(Xr, Yr);
      theta = RobotContainer.swerveSubsystem.getPose2d().getRotation().getRadians();
      phi = Math.atan2(Xr, Yr);

      robotX = RobotContainer.swerveSubsystem.getPose2d().getX();
      robotY = RobotContainer.swerveSubsystem.getPose2d().getY();
      
      SmartDashboard.putNumber("Theta", Math.toDegrees(theta));
      SmartDashboard.putNumber("Phi", Math.toDegrees(phi));


      tagOffsetX = (distanceToTarget * Math.sin(theta - phi));
      tagOffsetY = (distanceToTarget * Math.cos(theta - phi));
      tagOdoX = robotX - tagOffsetX;
      tagOdoY = robotY - tagOffsetY;

      SmartDashboard.putNumber("Xo Offset to Primary Tag", tagOffsetX);
      SmartDashboard.putNumber("Yo Offset to Primary Tag", tagOffsetY);
      SmartDashboard.putNumber("Tag X location in Absolute Odometer", tagOdoX);
      SmartDashboard.putNumber("Tag Y location in Absolute Odometer", tagOdoY);
  }
}
