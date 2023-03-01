// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.AutoConstants;
import frc.robot.utilities.*;

import com.revrobotics.SparkMaxRelativeEncoder;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class AUTOwaitUntilPIDtargetCMD extends CommandBase {
  /** Creates a new AUTOwaitUntilPIDtargetCMD. */
  public SparkMaxPIDControllerSmart pid;
  public boolean cmdComplete = false;
  public SparkMaxRelativeEncoder encoder;
  public double highTol;
  public double lowTol;
  public double setpoint;

  public AUTOwaitUntilPIDtargetCMD(SparkMaxPIDControllerSmart pidCon, SparkMaxRelativeEncoder encoder) {
    // Use addRequirements() here to declare subsystem dependencies.
    pid = pidCon;
    this.encoder = encoder;
    setpoint = pid.getSetpoint();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    setpoint = pid.getSetpoint();
    highTol = encoder.getPosition() + AutoConstants.kPIDtimerTol;
    lowTol = encoder.getPosition() - AutoConstants.kPIDtimerTol;

    cmdComplete = (setpoint >= lowTol && setpoint <= highTol);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return cmdComplete;
  }
}
