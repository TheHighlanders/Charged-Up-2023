// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonCMDs;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.vision;

public class VISIONalignAprilTag extends CommandBase {
  /** Creates a new VISIONalignAprilTag. */
  private vision vision;
  private SwerveSubsystem swerveSubsystem;

  private double targetX;
  private double targetY;
  boolean tagGood = false;
  boolean valid = false;
  double parkX;
  double parkY;
  AUTOswerveMoveCommand swerveMove;

  public VISIONalignAprilTag(double parkX, double parkY, vision vision_subsystem, SwerveSubsystem swerve_subsystem) {
    vision = vision_subsystem;
    swerveSubsystem = swerve_subsystem;
    this.parkX = parkX;
    this.parkY = parkY;
    addRequirements(swerveSubsystem, vision);


    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerveSubsystem.zeroHeading();
    swerveSubsystem.resetOdometry(new Pose2d(new Translation2d(), new Rotation2d()));
    if (vision.tvVal == 1) {
      targetX = vision.tagOdoX + parkX;
      targetY = vision.tagOdoY - AutoConstants.kAprilTagParkingDistance + parkY;
      tagGood = true;
      SmartDashboard.putNumber("X", targetX);
      SmartDashboard.putNumber("Y", targetY);
      swerveMove = new AUTOswerveMoveCommand(swerveSubsystem, targetX, -targetY, new Rotation2d(Math.toRadians(90)), true);
      swerveMove.initialize();

    }


    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(tagGood){swerveMove.execute();}
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveMove.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(tagGood){
      return swerveMove.isFinished();
    }
    return true;
  }
}
