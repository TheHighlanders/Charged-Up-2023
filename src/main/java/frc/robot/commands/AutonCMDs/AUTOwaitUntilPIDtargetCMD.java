// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonCMDs;

import frc.robot.utilities.*;

import com.revrobotics.RelativeEncoder;


import edu.wpi.first.wpilibj2.command.CommandBase;

public class AUTOwaitUntilPIDtargetCMD extends CommandBase {
  /** Creates a new AUTOwaitUntilPIDtargetCMD. */
  public SparkMaxPIDControllerSmart pid;
  public boolean cmdComplete = false;
  public RelativeEncoder encoder;
  double toleranceDeg;

  public AUTOwaitUntilPIDtargetCMD(SparkMaxPIDControllerSmart pidCon, RelativeEncoder encoder, double tolerance) {
    // Use addRequirements() here to declare subsystem dependencies.
    pid = pidCon;
    this.encoder = encoder;
    toleranceDeg = tolerance / 2;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double setpoint = pid.getSetpoint();
    double highTol = setpoint + toleranceDeg;
    double lowTol = setpoint - toleranceDeg;
    double position = encoder.getPosition();

    cmdComplete = (position >= lowTol) && (position <= highTol);

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
