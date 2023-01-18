// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class AUTOtrajectory extends CommandBase {
  /** Creates a new AUTOtrajectory. */
  public Command fullAuto;

  private SwerveSubsystem swerveSubsystem;

  public AUTOtrajectory(SwerveSubsystem swerve_subsytem) {
    swerveSubsystem = swerve_subsytem;
    addRequirements(swerveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // This will load the file "FullAuto.path" and generate it with a max velocity
    // of 4 m/s and a max acceleration of 3 m/s^2
    // for every path in the group
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("New Path", new PathConstraints(4, 3));
    DriverStation.reportWarning("autoTrajectory", false);
    // This is just an example event map. It would be better to have a constant,
    // global event map
    // in your code that will be used by all path following commands.
    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("marker1", new PrintCommand("Passed marker 1"));
    // eventMap.put("intakeDown", new IntakeDown());

    // Create the AutoBuilder. This only needs to be created once when robot code
    // starts, not every time you want to create an auto command. A good place to
    // put this is in RobotContainer along with your subsystems.
    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
        swerveSubsystem::getPose2d, // Pose2d supplier
        swerveSubsystem::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
        DriveConstants.kDriveKinematics, // SwerveDriveKinematics
        new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y
        // PID controllers)
        new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation
        // controller)
        swerveSubsystem::setModuleStates, // Module states consumer used to output to the drive subsystem
        eventMap,
        true,
        swerveSubsystem // Should the path be automatically mirrored depending on alliance color.
                                        // Optional, defaults to true
                                        // The drive subsystem. Used to properly set the requirements of path following
                                        // commands
    );

    fullAuto = autoBuilder.fullAuto(pathGroup);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
