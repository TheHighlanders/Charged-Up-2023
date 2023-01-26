// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.ArmCMDs.ArmToMiddleCMD;
import frc.robot.commands.ArmCMDs.ArmToStowedCMD;
import frc.robot.commands.ArmCMDs.ArmToTopCMD;
import frc.robot.commands.ArmCMDs.ArmtoShelfCMD;
import frc.robot.commands.AutonCMDs.AUTOhomeModulesCMD;
import frc.robot.commands.AutonCMDs.AUTOtrajectory;
import frc.robot.commands.GrabberCMDs.GrabberCloseCMD;
import frc.robot.commands.GrabberCMDs.GrabberOpenCMD;
import frc.robot.commands.IntakeCMDs.spinIntakeInCMD;
import frc.robot.commands.IntakeCMDs.spinIntakeOutCMD;
import frc.robot.commands.SwerveCMDs.SwerveJoystickCMD;
import frc.robot.commands.SwerveCMDs.ToggleFieldOrientedCMD;
import frc.robot.commands.SwerveCMDs.ZeroHeadingCMD;
import frc.robot.subsystems.AUTOsubsystem;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.Intake;
//import frc.robot.commands.encoderPrintout;
import frc.robot.subsystems.SwerveSubsystem;
import pabeles.concurrency.ConcurrencyOps.NewInstance;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
//import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

//import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;

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
  private final AUTOsubsystem auto = new AUTOsubsystem();
  private final AUTOtrajectory trajectory = new AUTOtrajectory(swerveSubsystem);
  private final XboxController driverJoystick = new XboxController(OIConstants.kdriverJoystick);
  private final XboxController operatorJoystick = new XboxController(OIConstants.koperatorJoystick);
 
  private final GrabberSubsystem grabberSub = new GrabberSubsystem();
  private final GrabberOpenCMD grabberOpen = new GrabberOpenCMD(grabberSub);
  private final GrabberCloseCMD grabberClose = new GrabberCloseCMD(grabberSub);

  private final Intake intakeSub = new Intake();
  private final spinIntakeInCMD spinIntakein = new spinIntakeInCMD(intakeSub);
  private final spinIntakeOutCMD spinIntakeOut = new spinIntakeOutCMD(intakeSub);

  private final Arm intakeArm = new Arm();
  private final ArmToMiddleCMD armMiddle = new ArmToMiddleCMD(intakeArm);
  private final ArmToStowedCMD armStowed = new ArmToStowedCMD(intakeArm);
  private final ArmToTopCMD armTop = new ArmToTopCMD(intakeArm);
  private final ArmtoShelfCMD armShelf = new ArmtoShelfCMD(intakeArm);


  // private final encoderPrintout encoderPrintoutCMD = new
  // encoderPrintout(swerveSubsystem);
  private final ZeroHeadingCMD zeroHeadingCMD = new ZeroHeadingCMD(swerveSubsystem);
  private final Command driveTrajectory = auto.getAuto();
  private final ToggleFieldOrientedCMD toggleFieldOrientedCMD = new ToggleFieldOrientedCMD(swerveSubsystem);

  private final SequentialCommandGroup autoGroup = new SequentialCommandGroup(
      new InstantCommand(() -> swerveSubsystem.zeroAllModules()),
      new InstantCommand(() -> swerveSubsystem.resetOdometry(new Pose2d())),
      new InstantCommand(
          () -> SmartDashboard.putString("Start Pose", swerveSubsystem.getPose2d() + " Start Pose")),
      driveTrajectory,
      new InstantCommand(() -> swerveSubsystem.stopModules()),
      new InstantCommand(
          () -> SmartDashboard.putString("End Pose", swerveSubsystem.getPose2d() + " Finsihed Pose")));
  // private final ExampleCommand m_autoCommand = new
  // ExampleCommand(m_exampleSubsystem);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCMD(swerveSubsystem));

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

    new JoystickButton(driverJoystick, 3).onTrue(new spinIntakeInCMD(intakeSub));//x
    new JoystickButton(driverJoystick, 4).onTrue(new spinIntakeOutCMD(intakeSub));//y

    new JoystickButton(operatorJoystick, 6).onTrue(new GrabberCloseCMD(grabberSub)); //RB
    new JoystickButton(operatorJoystick, 5).onTrue(new GrabberOpenCMD(grabberSub)); //LB
  


    new JoystickButton(operatorJoystick, 0).onTrue(new ArmToMiddleCMD(intakeArm));
    new JoystickButton(operatorJoystick, 0).onTrue(new ArmToStowedCMD(intakeArm));
    new JoystickButton(operatorJoystick, 0).onTrue(new ArmToTopCMD(intakeArm));
    new JoystickButton(operatorJoystick, 0).onTrue(new ArmtoShelfCMD(intakeArm));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    // Construct Auto Swerve Command Using Points and Objects from AUTOsubsystem
    // (auto)
    return autoGroup;
    // new AUTOhomeModulesCMD(swerveSubsystem),
    // new InstantCommand(()->
    // swerveSubsystem.resetOdometry(trajectory.getInitialPose())), // SOME
    // TRAJECTORY STUFF: TODO: MIGRATE TO ACTUAL COMMAND
    // swerveControllerCommand,

    // driveTrajectory,

  }
}
