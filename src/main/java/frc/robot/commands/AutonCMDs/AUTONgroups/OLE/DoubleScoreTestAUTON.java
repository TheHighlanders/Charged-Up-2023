// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonCMDs.AUTONgroups.OLE;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.ArmCMDs.ArmMoveCMD;
import frc.robot.commands.AutonCMDs.AUTOWaitCMD;
import frc.robot.commands.AutonCMDs.AUTOswerveMoveCommand;
import frc.robot.commands.AutonCMDs.AUTOwaitUntilPIDtargetCMD;
import frc.robot.commands.AutonCMDs.VISIONalignAprilTag;
import frc.robot.commands.AutonCMDs.autoBalanceCommand;
import frc.robot.commands.AutonCMDs.IntakeAUTO.AUTOstartIntakeCMD;
import frc.robot.commands.GrabberCMDs.GrabberCloseCMD;
import frc.robot.commands.GrabberCMDs.GrabberOpenCMD;
import frc.robot.commands.GrabberCMDs.GrabberPosCMD;
import frc.robot.commands.IntakeCMDs.DeployIntakeCMD;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.GyroSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.vision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DoubleScoreTestAUTON extends SequentialCommandGroup {
  /** Creates a new BalanceAUTON. */
  double standardTol = 5.0f*8100.0f/360.0f;
  public DoubleScoreTestAUTON(SwerveSubsystem swerveSubsystem, Arm armSubsystem, Intake intakeSubsystem, GrabberSubsystem grabberSubsystem, GyroSubsystem gyroSubsystem, vision visionSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> swerveSubsystem.resetOdometry(new Pose2d(new Translation2d(0,0), new Rotation2d(0)))),
      new InstantCommand(() -> swerveSubsystem.zeroHeading()),
      new GrabberCloseCMD(grabberSubsystem),
      new DeployIntakeCMD(intakeSubsystem, armSubsystem),
      new AUTOWaitCMD(1),
      new ArmMoveCMD(ArmConstants.kTopPos, armSubsystem, intakeSubsystem),
      new AUTOwaitUntilPIDtargetCMD(armSubsystem.armPID, armSubsystem.ArmEncoder, standardTol),
      new AUTOWaitCMD(2),
      new GrabberOpenCMD(grabberSubsystem),
      new AUTOWaitCMD(1),
      new GrabberPosCMD(grabberSubsystem),
      new ArmMoveCMD(ArmConstants.kStowedPos, armSubsystem, intakeSubsystem),
      new AUTOwaitUntilPIDtargetCMD(armSubsystem.armPID, armSubsystem.ArmEncoder, standardTol),
      new AUTOstartIntakeCMD(intakeSubsystem),
      // new DeployIntakeCMD(intakeSubsystem, armSubsystem), //Not Retracting Intake to Pickup Cube
      //new InstantCommand(() -> swerveSubsystem.setLastValidHeading(swerveSubsystem.getRotation2D().minus(new Rotation2d(Math.toRadians(90))))),
      new AUTOswerveMoveCommand(swerveSubsystem, 0, 4.25, new Rotation2d(Math.toRadians(180 + 90 /*Counter Offset for move */)), false),
      new DeployIntakeCMD(intakeSubsystem, armSubsystem),
      new AUTOswerveMoveCommand(swerveSubsystem, 0.0, 1.0, new Rotation2d(Math.PI / 2), true),
      new VISIONalignAprilTag(0, 0, visionSubsystem, swerveSubsystem),
      new DeployIntakeCMD(intakeSubsystem, armSubsystem),
      new GrabberCloseCMD(grabberSubsystem),
      new ArmMoveCMD(ArmConstants.kTopPos, armSubsystem, intakeSubsystem),
      new GrabberOpenCMD(grabberSubsystem)
    );
  }
}
