// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCMDs;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ArmtoShelfCMD extends CommandBase {
  /** Creates a new ArmToTopCMD. */

  public final Arm Arm_sub;

  public ArmtoShelfCMD(Arm arm_subArm) {
    // Use addRequirements() here to declare subsystem dependencies.
    Arm_sub = arm_subArm;
    addRequirements(Arm_sub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Arm_sub.shelf();
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
