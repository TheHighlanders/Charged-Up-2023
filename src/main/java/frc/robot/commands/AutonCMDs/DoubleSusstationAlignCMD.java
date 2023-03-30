// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonCMDs;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.ArmCMDs.ArmMoveCMD;
import frc.robot.commands.GrabberCMDs.GrabberPosCMD;
import frc.robot.commands.IntakeCMDs.DeployIntakeCMD;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DoubleSusstationAlignCMD extends SequentialCommandGroup {
  /** Creates a new DoubleSusstationAlignCMD. */
  public DoubleSusstationAlignCMD(SwerveSubsystem swerveSubsystem, Arm armSubsystem, Intake intakeSubsystem, GrabberSubsystem grabberSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> swerveSubsystem.resetOdometry(new Pose2d(new Translation2d(0,0), new Rotation2d(0)))),
      new AUTOswerveMoveCommand(swerveSubsystem, 0, 1, swerveSubsystem.getRotation2D().plus(new Rotation2d(Math.PI/2)), true),
      new DeployIntakeCMD(intakeSubsystem, armSubsystem),
      new AUTOWaitCMD(2),
      new ArmMoveCMD(ArmConstants.kShelfPos, armSubsystem, intakeSubsystem),
      new AUTOwaitUntilPIDtargetCMD(armSubsystem.armPID, armSubsystem.ArmEncoder, ArmConstants.kArmToleranceRotations),
      new AUTOWaitCMD(0.5),
      new GrabberPosCMD(grabberSubsystem),
      new AUTOswerveMoveCommand(swerveSubsystem, 0, 0.5, swerveSubsystem.getRotation2D().plus(new Rotation2d(Math.PI/2)), true)
    );
  }
}
