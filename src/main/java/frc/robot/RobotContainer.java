// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.opencv.osgi.OpenCVInterface;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.ArmCMDs.ArmDownCMD;
import frc.robot.commands.ArmCMDs.ArmMoveCMD;
import frc.robot.commands.ArmCMDs.ArmUpCMD;
import frc.robot.commands.AutonCMDs.AUTOWaitCMD;
import frc.robot.commands.AutonCMDs.AUTOcsvPathFollowCMD;
import frc.robot.commands.AutonCMDs.AUTOswerveMoveCommand;
import frc.robot.commands.AutonCMDs.DoubleSusstationAlignCMD;
import frc.robot.commands.AutonCMDs.VISIONalignAprilTag;
import frc.robot.commands.AutonCMDs.autoBalanceCommand;

import frc.robot.commands.AutonCMDs.AUTONgroups.HighMobilityEngageAUTO;
import frc.robot.commands.AutonCMDs.AUTONgroups.HybridCubeMobilityAUTON;
import frc.robot.commands.AutonCMDs.AUTONgroups.HybridMobilityEngageAUTO;
import frc.robot.commands.AutonCMDs.AUTONgroups.BackupAUTON;
import frc.robot.commands.AutonCMDs.AUTONgroups.BalanceAUTON;
import frc.robot.commands.AutonCMDs.AUTONgroups.ChargeStationAUTON;
import frc.robot.commands.AutonCMDs.AUTONgroups.LoadingZoneAUTON;
import frc.robot.commands.AutonCMDs.AUTONgroups.NoMoveAUTON;
import frc.robot.commands.AutonCMDs.AUTONgroups.ScoringTableAUTON;
import frc.robot.commands.AutonCMDs.AUTONgroups.TestAUTON;
import frc.robot.commands.IntakeCMDs.TurntableSpinCMD;
import frc.robot.commands.GrabberCMDs.GrabberCloseCMD;
import frc.robot.commands.GrabberCMDs.GrabberOpenCMD;
import frc.robot.commands.GrabberCMDs.GrabberZeroCMDG;
import frc.robot.commands.GrabberCMDs.GrabberPosCMD;
import frc.robot.commands.IntakeCMDs.DeployIntakeCMD;
import frc.robot.commands.IntakeCMDs.spinIntakeCMD;
import frc.robot.commands.SwerveCMDs.SlowSwerveCMD;
import frc.robot.commands.SwerveCMDs.SwerveJoystickCMD;
import frc.robot.commands.SwerveCMDs.ToggleFieldOrientedCMD;
import frc.robot.commands.SwerveCMDs.ZeroHeadingCMD;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.GyroSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SensorSubsystem;
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

  public static final Intake intakeSub = new Intake();

  private final Arm armSubsystem = new Arm();

  private final GyroSubsystem gyroSubsystem = new GyroSubsystem();

  String autoPath1 = "pathplanner/generatedCSV/ScoringTable1.csv";

  private final ZeroHeadingCMD zeroHeadingCMD = new ZeroHeadingCMD(swerveSubsystem);
  private final VISIONalignAprilTag visionAlignCMD = new VISIONalignAprilTag(0, 0.75, vision, swerveSubsystem);
  private final ToggleFieldOrientedCMD toggleFieldOrientedCMD = new ToggleFieldOrientedCMD(swerveSubsystem);

  private final AUTOcsvPathFollowCMD testingCSVtrajectory = new AUTOcsvPathFollowCMD(Filesystem.getDeployDirectory().toPath().resolve(autoPath1).toString(), swerveSubsystem);

  private final AUTOswerveMoveCommand swerveMove = new AUTOswerveMoveCommand(swerveSubsystem, 1, 1, new Rotation2d(Math.toRadians(90)), true);

  private final DeployIntakeCMD deployIntakeCMD = new DeployIntakeCMD(intakeSub, armSubsystem);
  private final spinIntakeCMD intakeInHighCMD = new spinIntakeCMD(IntakeConstants.kIntakeSpeedHigh, intakeSub);
  private final spinIntakeCMD intakeInLowCMD = new spinIntakeCMD(IntakeConstants.kIntakeSpeedLow, intakeSub);
  private final spinIntakeCMD intakeOutCMD = new spinIntakeCMD(-IntakeConstants.kIntakeSpeedHigh, intakeSub);

  private final TurntableSpinCMD spinTurntableCMD = new TurntableSpinCMD(intakeSub, false);
  private final TurntableSpinCMD spinTurntableReverseCMD = new TurntableSpinCMD(intakeSub, true);

  private final ArmMoveCMD armStowCMD = new ArmMoveCMD(ArmConstants.kStowedPos, armSubsystem, intakeSub);
  private final ArmMoveCMD armDownCMD = new ArmMoveCMD(ArmConstants.kDownPos, armSubsystem, intakeSub);
  private final ArmMoveCMD armMidCMD = new ArmMoveCMD(ArmConstants.kMiddlePos, armSubsystem, intakeSub);
  private final ArmMoveCMD armShelfCMD = new ArmMoveCMD(ArmConstants.kShelfPos, armSubsystem, intakeSub);
  private final ArmMoveCMD armTopCMD = new ArmMoveCMD(ArmConstants.kTopPos, armSubsystem, intakeSub);

  private final ArmUpCMD armMoveUpCMD = new ArmUpCMD(armSubsystem);
  private final ArmDownCMD armMoveDownCMD = new ArmDownCMD(armSubsystem);

  private final autoBalanceCommand balanceCMD = new autoBalanceCommand(gyroSubsystem, swerveSubsystem);

  //private final SequentialCommandGroup oneScoreAUTO = new BackupAUTON(swerveSubsystem, grabberSub, armSubsystem, intakeSub);

  private final SequentialCommandGroup scoringTableAUTO = new ScoringTableAUTON(swerveSubsystem, armSubsystem,
      grabberSub, intakeSub, vision);
  private final SequentialCommandGroup loadingZoneAUTO = new LoadingZoneAUTON(swerveSubsystem, armSubsystem,
  grabberSub, intakeSub, vision);

  private final SequentialCommandGroup testSubsystemsAUTO = new TestAUTON(armSubsystem, intakeSub, grabberSub);

  private final SequentialCommandGroup chargeStationAUTO = new ChargeStationAUTON(swerveSubsystem, armSubsystem, grabberSub, intakeSub, vision, gyroSubsystem);

  // A chooser for autonomous commands
  public static SendableChooser<SequentialCommandGroup> m_chooser = new SendableChooser<>();

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
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCMD(swerveSubsystem));
    m_chooser.setDefaultOption("Score CUBE HYBRID and Engage", new BalanceAUTON(swerveSubsystem, armSubsystem, intakeSub, grabberSub, gyroSubsystem));
    // m_chooser.addOption("Scoring", scoringTableAUTO);
    // m_chooser.addOption("Loading", loadingZoneAUTO);
    // m_chooser.addOption("Charge Station", chargeStationAUTO);
    // m_chooser.addOption("Test Trajectory DNS", new SequentialCommandGroup(testingCSVtrajectory));
    // m_chooser.addOption("Point Move CMDG DNS", new SequentialCommandGroup(swerveMove));
    // m_chooser.addOption("Subsystem Test DNS", testSubsystemsAUTO);
    m_chooser.addOption("Score CONE HIGH and Mobility",  new BackupAUTON(swerveSubsystem, grabberSub, armSubsystem, intakeSub));
    //m_chooser.addOption("Score Cube and Engage", new BalanceAUTON(swerveSubsystem, armSubsystem, intakeSub, grabberSub, gyroSubsystem));
    m_chooser.addOption("Nothing ", new SequentialCommandGroup(new AUTOWaitCMD(14)));
    m_chooser.addOption("No Move AUTO", new NoMoveAUTON(swerveSubsystem, armSubsystem, intakeSub, grabberSub, gyroSubsystem));
    //m_chooser.addOption("High Cube, Mobility, Engage", new HighMobilityEngageAUTO(swerveSubsystem, intakeSub, gyroSubsystem, grabberSub, armSubsystem));
    m_chooser.addOption("Score CUBE HYBRID and Mobility", new HybridCubeMobilityAUTON(swerveSubsystem, armSubsystem, intakeSub));
    
    SmartDashboard.putData(m_chooser);

    DriverStation.silenceJoystickConnectionWarning(true);

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
    new JoystickButton(driverJoystick, 8).onTrue(balanceCMD);
    new JoystickButton(driverJoystick, 7).whileTrue(intakeOutCMD);

    new JoystickButton(driverJoystick, 6).whileTrue(deployIntakeCMD);
    new JoystickButton(driverJoystick, 5).whileTrue(intakeInHighCMD);
    //new JoystickButton(driverJoystick, 6).whileTrue(intakeInLowCMD);
    //new POVButton(driverJoystick, 0).whileTrue(new VISIONalignAprilTag(0, 0, vision, swerveSubsystem));
    //new POVButton(driverJoystick, 90).whileTrue(new VISIONalignAprilTag(1, AutoConstants.kConeNodeOffsetMeters, vision, swerveSubsystem));
    //new POVButton(driverJoystick, 270).whileTrue(new VISIONalignAprilTag(1, -AutoConstants.kConeNodeOffsetMeters, vision, swerveSubsystem));

    new Trigger(() -> driverJoystick.getLeftTriggerAxis() > 0.5).whileTrue(new DoubleSusstationAlignCMD(swerveSubsystem, armSubsystem, intakeSub, grabberSub));
    new Trigger(() -> driverJoystick.getRightTriggerAxis() > 0.5).whileTrue(new SlowSwerveCMD(swerveSubsystem));

    new JoystickButton(operatorJoystick, 7).whileTrue(spinTurntableCMD);
    new JoystickButton(operatorJoystick, 9).whileTrue(spinTurntableReverseCMD);

    new JoystickButton(operatorJoystick, 5).onTrue(new GrabberCloseCMD(grabberSub));
    new JoystickButton(operatorJoystick, 6).onTrue(new GrabberOpenCMD(grabberSub)); //x=3

    new JoystickButton(operatorJoystick, 10).onTrue(new InstantCommand(()-> grabberSub.zero()));

    new Trigger(() -> operatorJoystick.getLeftTriggerAxis() > 0.5).whileTrue(new GrabberPosCMD(grabberSub));
    new Trigger(() -> operatorJoystick.getRightTriggerAxis() > 0.5).whileTrue(new GrabberZeroCMDG(grabberSub));

    new JoystickButton(operatorJoystick, 1).onTrue(armStowCMD);
    new JoystickButton(operatorJoystick, 2).onTrue(armDownCMD);
    new JoystickButton(operatorJoystick, 3).onTrue(armMidCMD);
    new JoystickButton(operatorJoystick, 4).onTrue(armShelfCMD);
    new JoystickButton(operatorJoystick, 8).onTrue(armTopCMD);

    new POVButton(operatorJoystick, 0).whileTrue(armMoveUpCMD);
    new POVButton(operatorJoystick, 180).whileTrue(armMoveDownCMD);

    // new JoystickButton(operatorJoystick, 7).whileTrue(new InstantCommand(() -> swerveSubsystem.jogModule(0.2, 0.2, 2)));
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

  public static void putAUTONchooser(){
    SmartDashboard.putData(m_chooser);
    // DriverStation.reportWarning("null", false);
  }
}
