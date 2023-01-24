// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SwerveCMDs;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class ZeroHeadingCMD extends CommandBase {
  /** Creates a new ZeroHeadingCMD. */
  private final SwerveSubsystem swerveSubsystem;

  public ZeroHeadingCMD(SwerveSubsystem swerve_subsystem) {
    swerveSubsystem = swerve_subsystem;
    addRequirements(swerve_subsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerveSubsystem.zeroHeading();
    //Possible need to set desired heading 0
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

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
