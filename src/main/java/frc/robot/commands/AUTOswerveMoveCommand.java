// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
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
  private boolean cmdDone = false;
  private double xEndPoint;
  private double yEndPoint;
  private Rotation2d headingEndPoint;

  private PIDController pidController = new PIDController(AutoConstants.kXPIDp, AutoConstants.kXPIDi,
      AutoConstants.kXPIDd);
  private double currentX;
  private double currentY;
  private Rotation2d currentHeading;

  private double deltaX;
  private double deltaY;
  private double deltaHeading;

  private double speedX;
  private double speedY;

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

    double pid = Math.abs(pidController.calculate(Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2)), 0))
        * AutoConstants.kMaxSpeedMetersPerSecond;

    speedX = (deltaX / (Math.abs(deltaX) + Math.abs(deltaY))) * pid;
    speedY = (deltaY / (Math.abs(deltaX) + Math.abs(deltaY))) * pid;

    cmdDone = !(Math.abs(deltaX) > AutoConstants.kTranslatePointError)
        && !(Math.abs(deltaY) > AutoConstants.kTranslatePointError)
        && !(Math.abs(deltaHeading) > AutoConstants.kRotationError);

    chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        (Math.abs(deltaX) > AutoConstants.kTranslatePointError ? speedX : 0.0),
        (Math.abs(deltaY) > AutoConstants.kTranslatePointError ? speedY : 0.0),
        (Math.abs(deltaHeading) > AutoConstants.kRotationError
            ? AutoConstants.kRotationSpeed * Math.signum(deltaHeading)
            : 0.0),
        swerveSubsystem.getRotation2D());

    //Putting Code to Drive

    //chassisSpeeds = swerveSubsystem.fieldOrientedThetaHold(chassisSpeeds);
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
    return cmdDone;
  }
}
