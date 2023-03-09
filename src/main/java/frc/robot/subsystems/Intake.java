// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  public VictorSPX intakeSpin = new VictorSPX(IntakeConstants.kIntakeMotorID);
  public VictorSPX intakeSpin2 = new VictorSPX(IntakeConstants.kIntakeMotor2ID);

  public TalonSRX intakeDeploy = new TalonSRX(IntakeConstants.kIntakeDeployID);
  public TalonSRX intakeDeploy2 = new TalonSRX(IntakeConstants.kIntakeDeploy2ID);

  public TalonSRX intakeTurnable = new TalonSRX(IntakeConstants.kIntakeTurntableID);

  public boolean deployed = false;
  public double currentSetpoint;

  /** Creates a new Intake. */
  public Intake() {
    intakeDeploy.selectProfileSlot(0, 0);

    intakeSpin2.setInverted(true);
    intakeSpin2.follow(intakeSpin);

    intakeDeploy2.setInverted(true);
    intakeDeploy2.follow(intakeDeploy);

    intakeDeploy.config_kP(IntakeConstants.DEPLOY_PID_ID, IntakeConstants.kPIntakeDeploy);
    intakeDeploy.config_kI(IntakeConstants.DEPLOY_PID_ID, IntakeConstants.kIIntakeDeploy);
    intakeDeploy.config_kD(IntakeConstants.DEPLOY_PID_ID, IntakeConstants.kDIntakeDeploy);
    // intakeDeploy2.config_kP(IntakeConstants.DEPLOY_PID_ID, IntakeConstants.kPIntakeDeploy);
    // intakeDeploy2.config_kI(IntakeConstants.DEPLOY_PID_ID, IntakeConstants.kIIntakeDeploy);
    // intakeDeploy2.config_kD(IntakeConstants.DEPLOY_PID_ID, IntakeConstants.kDIntakeDeploy);
  }

  public void deployIntake() {
    deployed = !deployed;
    DriverStation.reportWarning("Deployed: " + deployed, false);
    //intakeDeploy.set(ControlMode.Current, (deployed ? IntakeConstants.kIntakeInCurr : IntakeConstants.kIntakeOutCurr));
    
  }

  public void spinIntakeIn() {
    intakeSpin.set(ControlMode.PercentOutput, IntakeConstants.kIntakeSpeedLow);
  }

  public void spinIntakeOut() {
    intakeSpin.set(ControlMode.PercentOutput, IntakeConstants.kIntakeSpeedLow * -1);
  }
  
  public void spinIntake(double speed){
    intakeSpin.set(ControlMode.PercentOutput, speed);
  }

  public void stop() {
    intakeSpin.set(ControlMode.PercentOutput, 0);
  }

  public void spinTurntable(double speed) {
    intakeTurnable.set(ControlMode.PercentOutput, speed);
  }

  public void stopTurntable() {
    spinTurntable(0);
  }

  @Override
  public void periodic() {
    if(currentSetpoint < IntakeConstants.kIntakeInCurr && deployed){
      currentSetpoint += 10;
    }
    if(currentSetpoint > IntakeConstants.kIntakeOutCurr && !deployed){
      currentSetpoint -= 10;
    }

    //(deployed ? IntakeConstants.kIntakeInCurr : IntakeConstants.kIntakeOutCurr)

    intakeDeploy.set(ControlMode.Position, currentSetpoint);
    // if(intakeDeploy.getSelectedSensorVelocity() <= 0.1){
    //   intakeDeploy.set(ControlMode.PercentOutput, 0);
    // }
    
    // This method will be called once per scheduler run
  }
}
