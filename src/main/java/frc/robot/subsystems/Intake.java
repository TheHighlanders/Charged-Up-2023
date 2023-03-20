// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.EnumMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.RelativeEncoder;

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
  public IntakePos state = IntakePos.RETRACT;
  public double currentSetpoint = IntakeConstants.kIntakeInCurr;
  public boolean atSetpoint = false;
  /** Creates a new Intake. */
  public Intake() {
    atSetpoint = false;
    currentSetpoint = IntakeConstants.kIntakeInCurr;
    state = IntakePos.RETRACT;
    deployed = false;

    intakeDeploy.selectProfileSlot(0, 0);

    intakeSpin2.setInverted(true);
    intakeSpin2.follow(intakeSpin);

    intakeDeploy2.setInverted(true);
    intakeDeploy2.follow(intakeDeploy);

    intakeTurnable.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    intakeTurnable.config_kP(0,1);
    intakeTurnable.config_kI(0,0);
    intakeTurnable.config_kD(0,0);

    intakeDeploy.config_kP(IntakeConstants.DEPLOY_PID_ID, IntakeConstants.kPIntakeDeploy);
    intakeDeploy.config_kI(IntakeConstants.DEPLOY_PID_ID, IntakeConstants.kIIntakeDeploy);
    intakeDeploy.config_kD(IntakeConstants.DEPLOY_PID_ID, IntakeConstants.kDIntakeDeploy);

    SupplyCurrentLimitConfiguration limit = new SupplyCurrentLimitConfiguration(true, 20,0, 2);
    intakeDeploy.configSupplyCurrentLimit(limit);
    intakeDeploy2.configSupplyCurrentLimit(limit);
    // intakeDeploy2.config_kP(IntakeConstants.DEPLOY_PID_ID, IntakeConstants.kPIntakeDeploy);
    // intakeDeploy2.config_kI(IntakeConstants.DEPLOY_PID_ID, IntakeConstants.kIIntakeDeploy);
    // intakeDeploy2.config_kD(IntakeConstants.DEPLOY_PID_ID, IntakeConstants.kDIntakeDeploy);

    intakePosMap.put(IntakePos.DEPLOYED, IntakeConstants.kIntakeOutCurr);
    intakePosMap.put(IntakePos.RETRACT_ALT, IntakeConstants.kIntakeMidCurr);
    intakePosMap.put(IntakePos.RETRACT, IntakeConstants.kIntakeInCurr);
  }

  public void deployIntake(IntakePos stateTarget, boolean toggle) {
    if(toggle){deployed = !deployed;}
    //DriverStation.reportWarning("Deployed: " + deployed, false);

    state = stateTarget;
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
  public void spinTurntableDeg(double deg){
    double pos = deg * IntakeConstants.kTurnGearRatio * IntakeConstants.kTurnBeltRatio;
    intakeTurnable.setSelectedSensorPosition(0);
    //DriverStation.reportWarning("TurntableDeg: " + intakeTurnable.getSelectedSensorPosition(), false);

    intakeTurnable.set(ControlMode.Position, pos);
  }

  public void stopTurntable() {
    spinTurntable(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("Intake State", state.toString());
    stepToStatePos(state);
    
    if(state == IntakePos.DEPLOYED){
      atSetpoint = Math.abs(intakePosMap.get(IntakePos.DEPLOYED) - intakeDeploy.getSelectedSensorPosition()) <= (Math.abs(intakePosMap.get(IntakePos.DEPLOYED).doubleValue()/10)); 
    }else if (state == IntakePos.RETRACT){
      atSetpoint = Math.abs(intakePosMap.get(IntakePos.RETRACT) - intakeDeploy.getSelectedSensorPosition()) <= (Math.abs(intakePosMap.get(IntakePos.DEPLOYED).doubleValue()/10)); 
    } else{
      atSetpoint = false;
    }

    if(atSetpoint){
      intakeDeploy.set(ControlMode.PercentOutput, 0);
    } else{
      intakeDeploy.set(ControlMode.Position, currentSetpoint);
    }
    
    SmartDashboard.putBoolean("At Setpoint", atSetpoint);
    SmartDashboard.putBoolean("Deployed", deployed);
    SmartDashboard.putNumber("Setpoint", currentSetpoint);
    // This method will be called once per scheduler run
  }

  public void stepToStatePos(IntakePos state){
    if(currentSetpoint < intakePosMap.get(state)){
      currentSetpoint += 50;
    }
    if(currentSetpoint > intakePosMap.get(state)){
      currentSetpoint -= 50;
    }
  }

  public enum IntakePos{
    DEPLOYED,
    RETRACT,
    RETRACT_ALT
  }
  public EnumMap<IntakePos, Double> intakePosMap = new EnumMap<>(IntakePos.class);
  
  public void zeroIntake(){
    intakeDeploy.setSelectedSensorPosition(0);
  }
}
