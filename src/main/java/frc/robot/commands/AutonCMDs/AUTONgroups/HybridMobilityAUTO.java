// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonCMDs.AUTONgroups;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutonCMDs.AUTOWaitCMD;
import frc.robot.commands.AutonCMDs.AUTOswerveMoveCommand;
import frc.robot.commands.AutonCMDs.IntakeAUTO.AUTOstartIntakeCMD;
import frc.robot.commands.AutonCMDs.IntakeAUTO.AUTOstopIntakeCMD;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HybridMobilityAUTO extends SequentialCommandGroup {
  /** Creates a new HybridMobilityAUTO. */
  public HybridMobilityAUTO(Intake intakeSubsystem, SwerveSubsystem swerveSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    new InstantCommand(() -> swerveSubsystem.resetOdometry(new Pose2d(new Translation2d(0,0), new Rotation2d(0)))),
    new InstantCommand(() -> swerveSubsystem.zeroHeading()),

    new AUTOWaitCMD(1),
    new AUTOswerveMoveCommand(swerveSubsystem, 0, -Units.inchesToMeters(8), swerveSubsystem.getRotation2D().plus(new Rotation2d(Math.PI/2)), true),

    new AUTOstartIntakeCMD(intakeSubsystem),
    new AUTOWaitCMD(2),
    new AUTOstopIntakeCMD(intakeSubsystem),



    new AUTOswerveMoveCommand(swerveSubsystem, 0, 4.25 - Units.inchesToMeters(8), swerveSubsystem.getRotation2D().plus(new Rotation2d(Math.PI/2)), true)
    );
  }
}
