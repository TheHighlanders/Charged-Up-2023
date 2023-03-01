// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ArmCMDs.ArmMoveCMD;
import frc.robot.commands.AutonCMDs.AUTONgroups.ScoringTableAUTON;
import frc.robot.commands.GrabberCMDs.GrabberCloseCMD;
import frc.robot.commands.GrabberCMDs.GrabberOpenCMD;
import frc.robot.commands.IntakeCMDs.DeployIntakeCMD;
import frc.robot.commands.IntakeCMDs.spinIntakeInCMD;
import frc.robot.commands.IntakeCMDs.spinIntakeOutCMD;
import frc.robot.commands.SwerveCMDs.SwerveJoystickCMD;
import frc.robot.commands.SwerveCMDs.ToggleFieldOrientedCMD;
import frc.robot.commands.SwerveCMDs.ZeroHeadingCMD;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.vision;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final static SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  public final static vision vision = new vision();
  private final XboxController driverJoystick = new XboxController(OIConstants.kdriverJoystick);
  private final XboxController operatorJoystick = new XboxController(OIConstants.koperatorJoystick);

  private final GrabberSubsystem grabberSub = new GrabberSubsystem();

  private final Intake intakeSub = new Intake();

  private final Arm armSubsystem = new Arm();

  String autoPath1 = "pathplanner/generatedCSV/New New Path.csv";
  // private final encoderPrintout encoderPrintoutCMD = new
  // encoderPrintout(swerveSubsystem);
  private final ZeroHeadingCMD zeroHeadingCMD = new ZeroHeadingCMD(swerveSubsystem);
  // private final VISIONalignAprilTag visionAlignCMD = new VISIONalignAprilTag(0, 0.75, vision, swerveSubsystem);
  private final ToggleFieldOrientedCMD toggleFieldOrientedCMD = new ToggleFieldOrientedCMD(swerveSubsystem);

  private final DeployIntakeCMD deployIntakeCMD = new DeployIntakeCMD(intakeSub);
  private final spinIntakeInCMD intakeInCMD = new spinIntakeInCMD(intakeSub);
  private final spinIntakeOutCMD intakeOutCMD = new spinIntakeOutCMD(intakeSub);

  private final ArmMoveCMD armStowCMD = new ArmMoveCMD(ArmConstants.kStowedPos, armSubsystem, intakeSub);
  private final ArmMoveCMD armDownCMD = new ArmMoveCMD(ArmConstants.kDownPos, armSubsystem, intakeSub);
  private final ArmMoveCMD armMidCMD = new ArmMoveCMD(ArmConstants.kMiddlePos, armSubsystem, intakeSub);
  private final ArmMoveCMD armShelfCMD = new ArmMoveCMD(ArmConstants.kShelfPos, armSubsystem, intakeSub);
  private final ArmMoveCMD armTopCMD = new ArmMoveCMD(ArmConstants.kTopPos, armSubsystem, intakeSub);

  private final SequentialCommandGroup scoringTableAUTO = new ScoringTableAUTON(swerveSubsystem, armSubsystem,
      grabberSub, intakeSub, vision);

  // A chooser for autonomous commands
  SendableChooser<SequentialCommandGroup> m_chooser = new SendableChooser<>();

  // private final AUTOtrajectoryGenerate trajectory = new AUTOtrajectoryGenerate(swerveSubsystem,
  //     new double[] { 2 },
  //     new double[] { 0 },
  //     new double[] { 0 },
  //     new boolean[] { true });
  // private final SequentialCommandGroup autoGroup = new SequentialCommandGroup(
  //     new InstantCommand(() -> swerveSubsystem.zeroAllModules()),
  //     new InstantCommand(() -> swerveSubsystem.resetOdometry(new Pose2d())),
  //     new InstantCommand(
  //         () -> SmartDashboard.putString("Start Pose", swerveSubsystem.getPose2d() + " Start Pose")),
  //     //trajectory.generateTrajectory(),
  //     new AUTOcsvPathFollowCMD(Filesystem.getDeployDirectory().toPath().resolve(autoPath1).toString(), swerveSubsystem),
  //     new InstantCommand(() -> swerveSubsystem.stopModules()),
  //     new InstantCommand(
  //         () -> SmartDashboard.putString("End Pose", swerveSubsystem.getPose2d() + " Finsihed Pose")),
  //     new PrintCommand("AUTO DONE"));
  // private final ExampleCommand m_autoCommand = new
  // ExampleCommand(m_exampleSubsystem);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    //swerveSubsystem.setDefaultCommand(new SwerveJoystickCMD(swerveSubsystem));

    m_chooser.setDefaultOption("Nothing", new SequentialCommandGroup());
    m_chooser.addOption("Scoring", scoringTableAUTO);

    SmartDashboard.putData(m_chooser);
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(driverJoystick, 2).onTrue(zeroHeadingCMD);
    new JoystickButton(driverJoystick, 3).onTrue(new InstantCommand(() -> swerveSubsystem.resetOdometry(new Pose2d())));
    new JoystickButton(driverJoystick, 1).onTrue(toggleFieldOrientedCMD);

    new JoystickButton(operatorJoystick, 5).onTrue(new GrabberCloseCMD(grabberSub));
    new JoystickButton(operatorJoystick, 6).onTrue(new GrabberOpenCMD(grabberSub)); //x=3

    new JoystickButton(driverJoystick, 0).onTrue(deployIntakeCMD);
    new JoystickButton(driverJoystick, 0).whileTrue(intakeInCMD);
    new JoystickButton(driverJoystick, 0).whileTrue(intakeOutCMD);

    new JoystickButton(operatorJoystick, 1).onTrue(armStowCMD);
    new JoystickButton(operatorJoystick, 2).onTrue(armDownCMD);
    new JoystickButton(operatorJoystick, 3).onTrue(armMidCMD);
    new JoystickButton(operatorJoystick, 4).onTrue(armShelfCMD);
    new JoystickButton(operatorJoystick, 8).onTrue(armTopCMD);

    new JoystickButton(operatorJoystick, 7).whileTrue(new InstantCommand(() -> swerveSubsystem.jogModule(0.2, 0.2, 2)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();

    // Construct Auto Swerve Command Using Points and Objects from AUTOsubsystem
    // (auto)
    // new AUTOhomeModulesCMD(swerveSubsystem),
    // new InstantCommand(()->
    // swerveSubsystem.resetOdometry(trajectory.getInitialPose())), // SOME
    // swerveControllerCommand,

    // driveTrajectory,

  }
}
