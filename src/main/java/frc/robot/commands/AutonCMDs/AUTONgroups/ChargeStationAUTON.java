// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonCMDs.AUTONgroups;

import edu.wpi.first.math.geometry.Pose2d;
//import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.ArmCMDs.ArmMoveCMD;
import frc.robot.commands.AutonCMDs.AUTOcsvPathFollowCMD;
import frc.robot.commands.AutonCMDs.VISIONalignAprilTag;
import frc.robot.commands.AutonCMDs.autoBalanceCommand;
import frc.robot.commands.GrabberCMDs.GrabberCloseCMD;
import frc.robot.commands.GrabberCMDs.GrabberOpenCMD;
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
public class ChargeStationAUTON extends SequentialCommandGroup {
  /** Creates a new ScoringTableAUTON. */
  private SwerveSubsystem swerveSubsystem;
  private Arm armSubsystem;
  private GrabberSubsystem grabberSubsystem;
  private Intake intakeSubsystem;
  private vision visionSubsystem;
  private GyroSubsystem gyroSubsystem;
  private String ChargeStation1 = "pathplanner/generatedCSV/ChargeStation1.csv";

  public ChargeStationAUTON(SwerveSubsystem swerve_subsystem, Arm arm_subsystem, GrabberSubsystem grabber_subsystem,
      Intake intake_subsystem, vision vision_subsystem, GyroSubsystem gyro_subsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    swerveSubsystem = swerve_subsystem;
    armSubsystem = arm_subsystem;
    grabberSubsystem = grabber_subsystem;
    intakeSubsystem = intake_subsystem;
    visionSubsystem = vision_subsystem;
    gyroSubsystem = gyro_subsystem;
    addRequirements(swerveSubsystem, armSubsystem, grabberSubsystem, intakeSubsystem, visionSubsystem, gyroSubsystem);

    // double retreatX = swerveSubsystem.getPose2d().getX();
    // double retreatY = swerveSubsystem.getPose2d().getY() + 0.5;
    // Rotation2d retreatHeading = new Rotation2d(swerveSubsystem.getHeading());

    Pose2d startPose = AutoConstants.kChargeStationCubeNode;

    String path1 = Filesystem.getDeployDirectory().toPath().resolve(ChargeStation1).toString();

    addCommands(
        new InstantCommand(() -> swerveSubsystem.zeroAllModules()),
        

        new GrabberCloseCMD(grabberSubsystem), //Finalize grab of preload
        new ArmMoveCMD(ArmConstants.kTopPos, armSubsystem, intakeSubsystem), //Reach Up to Top
        new DeployIntakeCMD(intakeSubsystem, armSubsystem),
        new VISIONalignAprilTag(AutoConstants.kConeNodeOffsetMeters, 0, visionSubsystem, swerveSubsystem), //Park
        new InstantCommand(() -> swerveSubsystem.resetOdometry(startPose)),
        new GrabberOpenCMD(grabberSubsystem), //Drop Cone
        new ArmMoveCMD(ArmConstants.kDownPos, armSubsystem, intakeSubsystem), //Retract Arm
        new AUTOcsvPathFollowCMD(path1, swerveSubsystem),
        new autoBalanceCommand(gyroSubsystem, swerveSubsystem)
    );
      }
    }
