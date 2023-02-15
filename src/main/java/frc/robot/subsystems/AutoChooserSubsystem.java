// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.AutonCMDs.AUTONgroups.ScoringTableAUTON;

public class AutoChooserSubsystem extends SubsystemBase {
  /** Creates a new AutoChooserSubsystem. */
  HashMap<String, SequentialCommandGroup> autoLibrary;
  private SwerveSubsystem swerveSubsystem;
  private Arm armSubsystem;
  private Intake intakeSubsystem;
  private GrabberSubsystem grabberSubsystem;
  private vision visionSubsystem;

  public AutoChooserSubsystem(SwerveSubsystem swerve_subsystem, Arm arm_subsystem, Intake intake_subsystem,
      GrabberSubsystem grabber_subsystem, vision vision_subsystem) {
    swerveSubsystem = swerve_subsystem;
    armSubsystem = arm_subsystem;
    intakeSubsystem = intake_subsystem;
    grabberSubsystem = grabber_subsystem;
    visionSubsystem = vision_subsystem;

    //Add more Autos here
    autoLibrary.put("Scoring Table",
        new ScoringTableAUTON(swerveSubsystem, armSubsystem, grabberSubsystem, intakeSubsystem, visionSubsystem));
  }

  /**
   * Returns an auto based on the robots starting positio on the field
   * @param Alliance color, either "red", or "blue"
   * @param Position the position of the root on the field
   * where 0 is in front of the grid in front of DS 1, and 2 is the grid in front of DS 3
   * @return Sequential Command Group corresponding to the selected AUTON
   */
  public SequentialCommandGroup getAuto(String alliance, int positionNum) {
    return new SequentialCommandGroup();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
