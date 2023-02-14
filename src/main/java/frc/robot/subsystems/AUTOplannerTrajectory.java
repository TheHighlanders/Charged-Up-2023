// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.AutonCMDs.AUTOswerveMoveCommand;

public class AUTOplannerTrajectory extends SubsystemBase {
  /** Creates a new AUTOplannerTrajectory. */

  private double[] xPoints;
  private double[] yPoints;
  private double[] headingPoints;
  private boolean[] stopAtPoints;

  public AUTOplannerTrajectory() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setPoints(double[] x, double[] y, double[] headings, boolean[] stop) {
    xPoints = x;
    yPoints = y;
    headingPoints = headings;
    stopAtPoints = stop;
  }

}
