// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonCMDs.AUTONgroups;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.*;
import frc.robot.commands.ArmCMDs.ArmMoveCMD;
import frc.robot.commands.ArmCMDs.ArmToDownCMD;
import frc.robot.commands.ArmCMDs.ArmToStowedCMD;
import frc.robot.commands.ArmCMDs.ArmToTopCMD;
import frc.robot.commands.AutonCMDs.AUTOcsvPathFollowCMD;
import frc.robot.commands.AutonCMDs.AUTOswerveMoveCommand;
import frc.robot.commands.AutonCMDs.VISIONalignAprilTag;
import frc.robot.commands.AutonCMDs.IntakeAUTO.AUTOstartIntakeCMD;
import frc.robot.commands.AutonCMDs.IntakeAUTO.AUTOstopIntakeCMD;
import frc.robot.commands.GrabberCMDs.GrabberCloseCMD;
import frc.robot.commands.GrabberCMDs.GrabberOpenCMD;
import frc.robot.commands.IntakeCMDs.DeployIntakeCMD;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.vision;

import java.nio.file.Path;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoringTableAUTON extends SequentialCommandGroup {
  /** Creates a new ScoringTableAUTON. */
  private SwerveSubsystem swerveSubsystem;
  private Arm armSubsystem;
  private GrabberSubsystem grabberSubsystem;
  private Intake intakeSubsystem;
  private vision visionSubsystem;
  private String ScoringTable1 = "ScoringTable1";
  private String ScoringTable2 = "ScoringTable2";

  public ScoringTableAUTON(SwerveSubsystem swerve_subsystem, Arm arm_subsystem, GrabberSubsystem grabber_subsystem,
      Intake intake_subsystem, vision vision_subsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    swerveSubsystem = swerve_subsystem;
    armSubsystem = arm_subsystem;
    grabberSubsystem = grabber_subsystem;
    intakeSubsystem = intake_subsystem;
    visionSubsystem = vision_subsystem;
    addRequirements(swerveSubsystem, armSubsystem, grabberSubsystem, intakeSubsystem);

    double retreatX = swerveSubsystem.getPose2d().getX();
    double retreatY = swerveSubsystem.getPose2d().getY() + 0.5;
    Rotation2d retreatHeading = new Rotation2d(swerveSubsystem.getHeading());

    Pose2d startPose = AutoConstants.kScoringTableConeNode;

    String path1 = Filesystem.getDeployDirectory().toPath().resolve(ScoringTable1).toString();
    String path2 = Filesystem.getDeployDirectory().toPath().resolve(ScoringTable2).toString();

    addCommands(
        new InstantCommand(() -> swerveSubsystem.zeroAllModules()),
        new InstantCommand(() -> swerveSubsystem.resetOdometry(startPose)),

        new GrabberCloseCMD(grabberSubsystem), //Finalize grab of preload
        new ArmMoveCMD(ArmConstants.kTopPos, armSubsystem, intakeSubsystem), //Reach Up to Top
        new VISIONalignAprilTag(AutoConstants.kConeNodeOffsetMeters, 0, visionSubsystem, swerveSubsystem), //Park

        new GrabberOpenCMD(grabberSubsystem), //Drop Cone
        new AUTOswerveMoveCommand(swerveSubsystem, retreatX, retreatY, retreatHeading, true), //Retreat from Node
        new AUTOstartIntakeCMD(intakeSubsystem), //Start Intake
        new ArmMoveCMD(ArmConstants.kDownPos, armSubsystem, intakeSubsystem), //Retract Arm
        new AUTOcsvPathFollowCMD(path1, swerveSubsystem), //Drive To second Peice
        new AUTOstopIntakeCMD(intakeSubsystem), //Stop Intake
        new DeployIntakeCMD(intakeSubsystem), //Retract Intake
        new AUTOcsvPathFollowCMD(path2, swerveSubsystem), //Drive back to Align Point
        new ArmMoveCMD(ArmConstants.kStowedPos, armSubsystem, intakeSubsystem), //Go to pick up
        new GrabberCloseCMD(grabberSubsystem), //Pick up Game Peice
        new ArmMoveCMD(ArmConstants.kTopPos, armSubsystem, intakeSubsystem), //Go to place pos of arm
        new VISIONalignAprilTag(0, 0, visionSubsystem, swerveSubsystem), // Line up
        new GrabberOpenCMD(grabberSubsystem) //Drop

    );
  }
}
