// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.AUTOswerveMoveCommand;

public class AUTOtrajectoryGenerate extends CommandBase {
  private SwerveSubsystem swerveSubsystem;

  private double[] xPoints;
  private double[] yPoints;
  private Rotation2d[] headingPoints;
  private boolean[] stopAtPoints;

  /** Creates a new AUTOtrajectoryGenerate. */
  public AUTOtrajectoryGenerate(SwerveSubsystem swerve_subsystem, double[] x, double[] y, double[] headings,
      boolean[] stop) {
    swerveSubsystem = swerve_subsystem;
    addRequirements(swerve_subsystem);

    xPoints = x;
    yPoints = y;
    stopAtPoints = stop;
    headingPoints = new Rotation2d[headings.length];
    for (int i = 0; i < headings.length; i++) {
      headingPoints[i] = new Rotation2d(Math.toRadians(headings[i] + 90));
    }
    stopAtPoints[stopAtPoints.length - 1] = true;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  public SequentialCommandGroup generateTrajectory() {
    SequentialCommandGroup out = new SequentialCommandGroup();
    for (int i = 0; i < xPoints.length; i++) {
      out.addCommands(
          new AUTOswerveMoveCommand(swerveSubsystem, xPoints[i], yPoints[i], headingPoints[i], stopAtPoints[i]));
    }
    return out;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
