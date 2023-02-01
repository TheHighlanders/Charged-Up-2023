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

  private double Xo;
  private double Yo;
  private double beta;
  private double gamma;

  private double odometerOffsetX;
  private double odometerOffsetY;

  private double tagOdoX;
  private double tagOdoY;

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
      // if(vision.tidVal == 6 || vision.tidVal == 7 || vision.tidVal == 8){
      //   double parkHeading = VisionConstants.kBlueParkAngle;
      // }
      // if(vision.tidVal == 6 || vision.tidVal == 7 || vision.tidVal == 8){
      //   double parkHeading = VisionConstants.kRedParkAngle;
      // }
      beta = swerveSubsystem.getPose2d().getRotation().getRadians();
      beta = MathUtil.angleModulus(beta);
      SmartDashboard.putNumber("beta ", beta);
      gamma = vision.alpha - beta;
      gamma %= Math.PI / 2;
      SmartDashboard.putNumber("Gamma", gamma);

      Yo = vision.Z * cos(gamma);
      Xo = vision.Z * Math.sin(gamma);

      odometerOffsetX = Yo;
      odometerOffsetY = -Xo;
      tagOdoX = swerveSubsystem.getPose2d().getTranslation().getX() + odometerOffsetX;
      tagOdoY = swerveSubsystem.getPose2d().getTranslation().getY() - odometerOffsetY;
      ; //Borken

      SmartDashboard.putNumber("Xo Offset to Primary Tag", Xo);
      SmartDashboard.putNumber("Yo Offset to Primary Tag", Yo);
      SmartDashboard.putNumber("Tag X location in Absolute Odometer", tagOdoX);
      SmartDashboard.putNumber("Tag Y location in Absolute Odometer", tagOdoY);
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
