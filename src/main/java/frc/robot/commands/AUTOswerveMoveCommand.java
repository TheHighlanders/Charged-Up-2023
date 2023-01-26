// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class AUTOswerveMoveCommand extends CommandBase {
  /** Creates a new AUTOswerveMoveCommad. */
  private SwerveSubsystem swerveSubsystem;

  private double xEndPoint;
  private double yEndPoint;
  private Rotation2d headingEndPoint;

  private double currentX;
  private double currentY;
  private Rotation2d currentHeading;

  private double deltaX;
  private double deltaY;
  private double deltaHeading;

  private Pose2d currentPose;

  private ChassisSpeeds chassisSpeeds;

  public AUTOswerveMoveCommand(SwerveSubsystem swerve_subsystem, double x, double y, Rotation2d heading) {
    // Use addRequirements() here to declare subsystem dependencies.
    swerveSubsystem = swerve_subsystem;
    addRequirements(swerveSubsystem);

    xEndPoint = x;
    yEndPoint = y;
    headingEndPoint = heading;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentPose = swerveSubsystem.getPose2d();

    currentX = currentPose.getX();
    currentY = currentPose.getY();
    currentHeading = currentPose.getRotation();

    deltaX = xEndPoint - currentX;
    deltaY = yEndPoint - currentY;

    deltaHeading = headingEndPoint.getRadians() - currentHeading.getRadians();

    chassisSpeeds = new ChassisSpeeds(
        (Math.abs(deltaX) > AutoConstants.kTranslatePointError * Math.signum(deltaX) ? AutoConstants.kTranslateSpeed
            : 0.0),
        0, //(Math.abs(deltaY) > AutoConstants.kTranslatePointError * Math.signum(deltaY) ? AutoConstants.kTranslateSpeed : 0.0),
        0 //(deltaHeading > AutoConstants.kRotationError * Math.signum(deltaHeading) ? AutoConstants.kRotationSpeed : 0.0)
    );

    //Putting Code to Drive
    chassisSpeeds = swerveSubsystem.fieldOrientedThetaHold(chassisSpeeds);
    SmartDashboard.putNumber("AUTO X chassisSpeeds", chassisSpeeds.vxMetersPerSecond);
    SwerveModuleState[] moduleStates = swerveSubsystem.getIKMathSwerveModuleStates(chassisSpeeds);

    swerveSubsystem.setModuleStates(moduleStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
