// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonCMDs.AUTONgroups;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.ArmCMDs.ArmMoveCMD;
import frc.robot.commands.AutonCMDs.AUTOWaitCMD;
import frc.robot.commands.AutonCMDs.AUTOswerveMoveCommand;
import frc.robot.commands.AutonCMDs.AUTOwaitUntilPIDtargetCMD;
import frc.robot.commands.AutonCMDs.autoBalanceCommand;
import frc.robot.commands.AutonCMDs.IntakeAUTO.AUTOstartIntakeCMD;
import frc.robot.commands.AutonCMDs.IntakeAUTO.AUTOstopIntakeCMD;
import frc.robot.commands.GrabberCMDs.GrabberCloseCMD;
import frc.robot.commands.GrabberCMDs.GrabberOpenCMD;
import frc.robot.commands.IntakeCMDs.DeployIntakeCMD;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.GyroSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HybridCubeMobilityAUTON extends SequentialCommandGroup {
  /** Creates a new BalanceAUTON. */
  public HybridCubeMobilityAUTON(SwerveSubsystem swerveSubsystem, Arm armSubsystem, Intake intakeSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> swerveSubsystem.resetOdometry(new Pose2d(new Translation2d(0,0), new Rotation2d(0)))),
      new InstantCommand(() -> swerveSubsystem.zeroHeading()),
      // new GrabberCloseCMD(grabberSubsystem),
      // new DeployIntakeCMD(intakeSubsystem, armSubsystem),
      // new AUTOWaitCMD(1),
      // new ArmMoveCMD(ArmConstants.kTopPos, armSubsystem, intakeSubsystem),
      // new AUTOWaitCMD(2),
      // new DeployIntakeCMD(intakeSubsystem, armSubsystem),
      // new AUTOWaitCMD(1),
      // new GrabberOpenCMD(grabberSubsystem),
      // new AUTOWaitCMD(1),
      // new GrabberCloseCMD(grabberSubsyste;;ppz pp1 ),dffffffffffffffffffffffffff5e
      new AUTOWaitCMD(2),
      new AUTOstopIntakeCMD(intakeSubsystem),
      //new ArmMoveCMD(ArmConstants.kDownPos, armSubsystem, intakeSubsystem),
      new AUTOWaitCMD(0.375),
      // // new InstantCommand(() -> swerveSubsystem.setLastValidHeading(swerveSubsystem.getRotation2D().minus(new Rotation2d(Math.toRadians(90))))),
      new InstantCommand(()-> SmartDashboard.putString("Start Rot", swerveSubsystem.getRotation2D().toString())),
      new AUTOswerveMoveCommand(swerveSubsystem, 0, 3.5, swerveSubsystem.getRotation2D().plus(new Rotation2d(Math.PI/2)), false)
    );
  }
}
