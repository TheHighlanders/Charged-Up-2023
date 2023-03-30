// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonCMDs;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.GyroConstants;
import frc.robot.subsystems.GyroSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class autoBalanceCommand extends CommandBase {
  private GyroSubsystem gyroSubsystem;
  private SwerveSubsystem swerveSubsystem;
  double initialAng;
  boolean clunked;
  boolean balanced;
  SwerveModuleState[] states;
  int driveBack = 0;
  double speed;
  int count;

  PIDController balancePID;
  /** Creates a new autoBalanceCommand. */
  public autoBalanceCommand(GyroSubsystem gyro_subsystem, SwerveSubsystem swerve_Subsystem) {
    gyroSubsystem = gyro_subsystem;
    swerveSubsystem = swerve_Subsystem;
    addRequirements(gyroSubsystem);
    addRequirements(swerveSubsystem);

    balancePID = new PIDController(GyroConstants.kPbal, GyroConstants.kIbal, GyroConstants.kDbal);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerveSubsystem.setLastValidHeading(new Rotation2d(swerveSubsystem.getHeading()));
    gyroSubsystem.balanced = false;
    clunked = false;
    count = 0;
    driveBack = 0;
    gyroSubsystem.CalculateAngle();
    initialAng = gyroSubsystem.getAngleDeg();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    // if((Math.abs(gyroSubsystem.getAngleDeg()) >= 13) && 
    // (initialAng * gyroSubsystem.getAngleDeg() < 0)){
    //   clunked = true;
    // }
    // speed = 0.1 * Math.signum(gyroSubsystem.angle);

    // if(count < 50){
    //   drive(speed);
    // } else{
    //   drive(0);
    // } 
    // count++;
    // count %= 75;
    double gyroDead = (Math.abs(gyroSubsystem.getAngleDeg())  <= 2.5 ? 0.0 : gyroSubsystem.getAngleDeg());
    drive(balancePID.calculate(gyroDead, 0) * -1 * GyroConstants.kBalMaxQuestionMark);

    gyroSubsystem.CalculateAngle();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  public void drive(double speed){
    SwerveModuleState[] states = new SwerveModuleState[4];
    for(int i = 0; i < 4; i++){
      states[i] = new SwerveModuleState(speed, new Rotation2d());

    }
    swerveSubsystem.setModuleStates(states);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return gyroSubsystem.balanced;
  }
}
