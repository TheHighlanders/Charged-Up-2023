// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonCMDs.AUTONgroups;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
public class HighMobilityEngageAUTO extends SequentialCommandGroup {
  /** Creates a new HybridMobilityEngageAUTO. */
  public HighMobilityEngageAUTO(SwerveSubsystem swerveSubsystem, Intake intakeSubsystem, GyroSubsystem gyroSubsystem, GrabberSubsystem grabberSubsystem, Arm armSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      /*Robot Prep */
      new InstantCommand(() -> swerveSubsystem.resetOdometry(new Pose2d(new Translation2d(0,0), new Rotation2d(0)))),
      new InstantCommand(() -> swerveSubsystem.zeroHeading()),

      /* Begin Robot Config for Place*/
      new GrabberCloseCMD(grabberSubsystem), //Grip Game Piece
      new DeployIntakeCMD(intakeSubsystem, armSubsystem), // Intake Out
      new AUTOWaitCMD(1.5), // Clearance for Intake exit collison zone - May need shortening
      new ArmMoveCMD(ArmConstants.kTopPos, armSubsystem, intakeSubsystem), // Arm High
      new AUTOwaitUntilPIDtargetCMD(armSubsystem.armPID, armSubsystem.ArmEncoder, ArmConstants.kArmToleranceRotations), // Wait Sequence
      new AUTOWaitCMD(2), 
      new DeployIntakeCMD(intakeSubsystem, armSubsystem), // Retract Intake to RETRACT_ALT
      new AUTOWaitCMD(1), // May need to be shorter to make time
      /*End Config for Place */

      /*Approach */
      new AUTOswerveMoveCommand(swerveSubsystem, 0, -Units.inchesToMeters(8), swerveSubsystem.getRotation2D().plus(new Rotation2d(Math.PI/2)), true),
      
      /*Place*/
      new GrabberOpenCMD(grabberSubsystem),
      new AUTOWaitCMD(2),
      new GrabberCloseCMD(grabberSubsystem),

      /*Retreat */
      new AUTOswerveMoveCommand(swerveSubsystem, 0, 0.25,  swerveSubsystem.getRotation2D().plus(new Rotation2d(Math.PI/2)), true),

      /*Reconfigure Robot State */
      new DeployIntakeCMD(intakeSubsystem, armSubsystem),
      
      new AUTOWaitCMD(0.5),

      new ArmMoveCMD(ArmConstants.kStowedPos, armSubsystem, intakeSubsystem),
      new AUTOwaitUntilPIDtargetCMD(armSubsystem.armPID, armSubsystem.ArmEncoder, ArmConstants.kArmToleranceRotations),
      
      new DeployIntakeCMD(intakeSubsystem, armSubsystem),

      /* Move Across Station */
      new AUTOswerveMoveCommand(swerveSubsystem, 0, 4.5, swerveSubsystem.getRotation2D().plus(new Rotation2d(Math.PI/2)), true),

      
      
      /*Re-engage with Charge Station */
      new AUTOswerveMoveCommand(swerveSubsystem, 0, 2 - Units.inchesToMeters(8), swerveSubsystem.getRotation2D().plus(new Rotation2d(Math.PI/2)), true)

      /*Complete the Balance */
      // new autoBalanceCommand(gyroSubsystem, swerveSubsystem)

    );
  }
}
