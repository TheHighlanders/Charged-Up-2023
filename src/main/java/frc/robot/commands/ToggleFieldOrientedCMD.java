// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class ToggleFieldOrientedCMD extends CommandBase {
  /** Creates a new ToggleFieldOrientedCMD. */
  private final SwerveSubsystem swerveSubsystem;

  public ToggleFieldOrientedCMD(SwerveSubsystem swerve_subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    swerveSubsystem = swerve_subsystem;
    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerveSubsystem.toggleFieldOrient();
    DriverStation.reportWarning("Toggled Swerve FoR " + swerveSubsystem.getFieldOrient(), false);
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
    return true;
  }
}
