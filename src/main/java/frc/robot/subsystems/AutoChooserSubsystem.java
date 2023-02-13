// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AutoChooserSubsystem extends SubsystemBase {
  /** Creates a new AutoChooserSubsystem. */
  public SequentialCommandGroup[] redAutons = new SequentialCommandGroup[] {};
  public SequentialCommandGroup[] blueAutons = new SequentialCommandGroup[] {};

  public AutoChooserSubsystem() {
  }

  /**
   * Returns an auto based on the robots starting positio on the field
   * @param Alliance color, either "red", or "blue"
   * @param Position the position of the root on the field
   * where 0 is in front of the grid in front of DS 1, and 2 is the grid in front of DS 3
   * @return Sequential Command Group corresponding to the selected AUTON
   */
  public SequentialCommandGroup getAuto(String alliance, int positionNum) {
    if (alliance.equals("red")) {
      return redAutons[positionNum];
    }
    if (alliance.equals("blue")) {
      return blueAutons[positionNum];
    }

    return new SequentialCommandGroup();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
