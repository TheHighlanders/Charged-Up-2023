// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonCMDs;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.vision;
import static java.lang.Math.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;

public class VISIONalignAprilTag extends CommandBase {
  /** Creates a new VISIONalignAprilTag. */
  private vision vision;
  private SwerveSubsystem swerveSubsystem;
  private boolean cmdDone = false;

  private double targetX;
  private double targetY;
  private Rotation2d targetTheta;

  double parkX;
  double parkY;

  public VISIONalignAprilTag(double parkX, double parkY, vision vision_subsystem, SwerveSubsystem swerve_subsystem) {
    vision = vision_subsystem;
    swerveSubsystem = swerve_subsystem;
    this.parkX = parkX;
    this.parkX = parkY;
    addRequirements(swerveSubsystem, vision);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (vision.tvVal == 1) {
      targetX = vision.tagOdoX + parkX;
      targetY = vision.tagOdoY + AutoConstants.kAprilTagParkingDistance + parkY;

      targetTheta = swerveSubsystem.getRotation2D();

    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    DriverStation.reportWarning("April Execute", false);
    swerveSubsystem.driveAUTOfieldOrient(targetX, targetY, new Rotation2d(Math.toRadians(90)));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return swerveSubsystem.cmdDone;
  }
}
