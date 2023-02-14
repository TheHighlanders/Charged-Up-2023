// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.IntakeConstants;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

public class Intake extends SubsystemBase {
  public TalonSRX intakeSpin = new TalonSRX(IntakeConstants.INTAKE_MOTOR_ID);
  public TalonSRX intakeDeploy = new TalonSRX(IntakeConstants.INTAKE_DEPLOY_ID);

  public TalonSRXConfiguration spinConfig = new TalonSRXConfiguration();
  public TalonSRXConfiguration deployConfig = new TalonSRXConfiguration();

  public boolean deployed = false;

  /** Creates a new Intake. */
  public Intake() {
    intakeDeploy.selectProfileSlot(0, 0);

    intakeDeploy.config_kP(IntakeConstants.DEPLOY_PID_ID, IntakeConstants.kPIntakeDeploy);
    intakeDeploy.config_kI(IntakeConstants.DEPLOY_PID_ID, IntakeConstants.kIIntakeDeploy);
    intakeDeploy.config_kD(IntakeConstants.DEPLOY_PID_ID, IntakeConstants.kDIntakeDeploy);
  }

  public void deployIntake() {
    deployed = !deployed;
    intakeDeploy.set(ControlMode.Position, (deployed ? IntakeConstants.kIntakeOutPos : IntakeConstants.kIntakeOutPos));
  }

  public void spinIntakeIn() {
    intakeSpin.set(ControlMode.PercentOutput, IntakeConstants.kIntakeSpeed);
  }

  public void spinIntakeOut() {
    intakeSpin.set(ControlMode.PercentOutput, IntakeConstants.kIntakeSpeed * -1);
  }

  public void stop() {
    intakeSpin.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}