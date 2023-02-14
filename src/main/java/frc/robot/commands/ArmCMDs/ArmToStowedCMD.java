// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCMDs;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

public class ArmToStowedCMD extends CommandBase {
  /** Creates a new ArmToStowedCMD. */

  public final Arm Arm_sub;
  private Intake intakeSubsystem;

  public ArmToStowedCMD(Arm arm_sub, Intake intake_subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    Arm_sub = arm_sub;
    intakeSubsystem = intake_subsystem;
    addRequirements(Arm_sub, intake_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (intakeSubsystem.deployed) {
      intakeSubsystem.deployIntake();
    }
    Arm_sub.stowed();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Arm_sub.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
