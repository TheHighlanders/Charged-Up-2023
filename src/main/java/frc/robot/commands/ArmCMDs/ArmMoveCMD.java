// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCMDs;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakePos;

public class ArmMoveCMD extends CommandBase {
  /** Creates a new ArmMoveCMD. */
  private Intake intakeSubsystem;
  private Arm armSubsystem;
  private double targetPos;

  public ArmMoveCMD(double targetPos, Arm arm_sub, Intake intake_sub) {
    intakeSubsystem = intake_sub;
    armSubsystem = arm_sub;
    this.targetPos = targetPos;

    addRequirements(arm_sub, intake_sub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double armPos = armSubsystem.ArmEncoder.getPosition();

    boolean armMovingThroughZone = false;

    if (armPos <= ArmConstants.kIntakeDeathZone) {
      armMovingThroughZone = true;
    } else if (targetPos <= ArmConstants.kIntakeDeathZone) {
      armMovingThroughZone = true;
    }

    if (armMovingThroughZone && !intakeSubsystem.deployed) {
      intakeSubsystem.deployIntake(IntakePos.DEPLOYED);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armSubsystem.moveToPos(targetPos);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // DriverStation.reportWarning("ArmMoveIsFinished", false);
    return true;
  }
}
