// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.SwerveCMDs.NothingSwerveCMD;
import frc.robot.commands.SwerveCMDs.SwerveJoystickCMD;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  //private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  //private final encoderPrintout encoderPrintoutCMD = new encoderPrintout(swerveSubsystem);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    m_robotContainer = new RobotContainer();

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    /*if (driverJoystick.getBackButtonPressed()) {
      camToogle = !camToogle;

      if (camToogle) {
        cameraSelection.setString(camera2.getName());
      } else {
        cameraSelection.setString(camera1.getName());
      }
    }*/

    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    RobotContainer.putAUTONchooser();
    // SmartDashboard.putString("Stored Odometry Pose",
    //     m_robotContainer.swerveSubsystem.odoPose.getTranslation().toString());
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    RobotContainer.swerveSubsystem.setDefaultCommand(new NothingSwerveCMD(RobotContainer.swerveSubsystem));
    RobotContainer.intakeSub.zeroIntake();
    RobotContainer.swerveSubsystem.zeroAllModules();
    RobotContainer.swerveSubsystem.zeroHeading();
    RobotContainer.swerveSubsystem.resetOdometry(new Pose2d(new Translation2d(), new Rotation2d()));
    // if(DriverStation.getAlliance() == DriverStation.Alliance.Blue){
    //   AutoConstants.kScoringTableConeNode = new Pose2d(new Translation2d(1.85, 0.45), new Rotation2d(0));
    //   AutoConstants.kLoadingZoneConeNode  = new Pose2d(new Translation2d(1.85, 4.95), new Rotation2d(0));
    //    AutoConstants.kChargeStationCubeNode = new Pose2d(new Translation2d(1.85, 2.75), new Rotation2d(0));
    // }
    // if(DriverStation.getAlliance() == DriverStation.Alliance.Red){
    //   AutoConstants.kScoringTableConeNode = new Pose2d(new Translation2d(14.75, 0.5), new Rotation2d(Math.toRadians(180)));
    //   AutoConstants.kLoadingZoneConeNode  = new Pose2d(new Translation2d(14.75, 5), new Rotation2d(Math.toRadians(180)));
    //   AutoConstants.kChargeStationCubeNode = new Pose2d(new Translation2d(14.75, 2.75), new Rotation2d(Math.toRadians(180)));
    // }
    
      

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    RobotContainer.swerveSubsystem.encoderPrintoutDeg();
  }

  @Override

  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
      RobotContainer.swerveSubsystem.zeroAllModules(); //MAY NEED TO CHANGE BC CUSTOM ABSOL ENCOD
      RobotContainer.swerveSubsystem.setDefaultCommand(new SwerveJoystickCMD(RobotContainer.swerveSubsystem));
      RobotContainer.swerveSubsystem.resetOdometry(new Pose2d());

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    RobotContainer.swerveSubsystem.encoderPrintoutDeg();
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();

  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    RobotContainer.swerveSubsystem.encoderPrintoutDeg();
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
