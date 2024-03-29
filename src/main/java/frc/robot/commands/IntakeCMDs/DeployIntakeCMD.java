// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCMDs;

import java.sql.Driver;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakePos;

public class DeployIntakeCMD extends CommandBase {
  /** Creates a new DeployIntakeCMD. */
  private Intake intakeSubsystem;
  private Arm armSubsystem;

  public DeployIntakeCMD(Intake intake_subsystem, Arm arm_subsystem) {
    intakeSubsystem = intake_subsystem;
    armSubsystem = arm_subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    IntakePos state = IntakePos.RETRACT;
    double armPos = armSubsystem.ArmEncoder.getPosition();

    DriverStation.reportWarning("DeployBeginning" + intakeSubsystem.deployed, false);

    if(!intakeSubsystem.deployed){
      state = IntakePos.DEPLOYED;
    }
    if(intakeSubsystem.deployed == true && 
        (armPos <= ArmConstants.kIntakeDeathZone && armPos >= ArmConstants.kIntakeDeathZoneLow)){
      return;
    }
    if(intakeSubsystem.deployed == true && 
        (armPos <= ArmConstants.kIntakeDeathZoneHighTop && armPos >= ArmConstants.kIntakeDeathZoneHighBottom)){
      state = IntakePos.RETRACT_ALT;
    }
    DriverStation.reportWarning("Commanded to State" + state.toString(), false);
    //DriverStation.reportWarning("DeployCMD", false);
    intakeSubsystem.deployIntake(state, true);
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
    return true;
  }
}
