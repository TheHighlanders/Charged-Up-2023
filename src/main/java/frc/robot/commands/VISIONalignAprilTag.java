// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.vision;
import static java.lang.Math.*;
import edu.wpi.first.math.MathUtil;

public class VISIONalignAprilTag extends CommandBase {
  /** Creates a new VISIONalignAprilTag. */
  private vision vision;
  private SwerveSubsystem swerveSubsystem;
  private boolean cmdDone = false;

  private double robotX;
  private double robotY;
  private double theta;
  private double phi;

  private double distanceToTarget;

  private double odometerOffsetX;
  private double odometerOffsetY;

  private double tagOdoX;
  private double tagOdoY;
  private double tagOffsetX;
  private double tagOffsetY;

  public VISIONalignAprilTag(vision vision_subsystem, SwerveSubsystem swerve_subsystem) {
    vision = vision_subsystem;
    swerveSubsystem = swerve_subsystem;
    addRequirements(swerveSubsystem, vision);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (vision.tvVal == 1) {
      //swerveSubsystem.driveAUTOnonFieldOrient(vision.tagOdoX, vision.tagOdoY, swerveSubsystem.getRotation2D());
      
    }
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
