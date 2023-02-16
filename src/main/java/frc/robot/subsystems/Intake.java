// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  public VictorSPX intakeSpin = new VictorSPX(IntakeConstants.INTAKE_MOTOR_ID);
  public VictorSPX intakeSpin2 = new VictorSPX(IntakeConstants.INTAKE_MOTOR2_ID);

  public TalonSRX intakeDeploy = new TalonSRX(IntakeConstants.INTAKE_DEPLOY_ID);
  public TalonSRX intakeDeploy2 = new TalonSRX(IntakeConstants.INTAKE_DEPLOY2_ID);

  public TalonSRXConfiguration spinConfig = new TalonSRXConfiguration();
  public TalonSRXConfiguration deployConfig = new TalonSRXConfiguration();

  public boolean deployed = false;

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

    intakeDeploy2.config_kP(IntakeConstants.DEPLOY_PID_ID, IntakeConstants.kPIntakeDeploy);
    intakeDeploy2.config_kI(IntakeConstants.DEPLOY_PID_ID, IntakeConstants.kIIntakeDeploy);
    intakeDeploy2.config_kD(IntakeConstants.DEPLOY_PID_ID, IntakeConstants.kDIntakeDeploy);
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
