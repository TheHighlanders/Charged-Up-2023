// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.ArmConstants;
import frc.robot.utilities.CANSparkMaxCurrent;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.SparkMaxPIDControllerSmart;

public class Arm extends SubsystemBase {
  public CANSparkMaxCurrent ArmMotor;
  public RelativeEncoder ArmEncoder;
  
  public double target;
  public double setpoint;

  public ControlType PIDposition = ControlType.kPosition;

  public SparkMaxPIDControllerSmart armPID;
  public SparkMaxPIDController armPIDunbox;

  //Torque needed to fight gravity = Distance of Cm * (cos(theta) * (9.81 * mass))

  /** Creates a new Arm. */
  public Arm() {
    ArmMotor = new CANSparkMaxCurrent(ArmConstants.ARM_MOTOR_ID, MotorType.kBrushless);
    ArmEncoder = ArmMotor.getEncoder();
    ArmEncoder.setPositionConversionFactor(ArmConstants.kArmRatio);
    ArmEncoder.setPosition(0.0);
    ArmMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    armPIDunbox = ArmMotor.getPIDController();
    armPID = new SparkMaxPIDControllerSmart(armPIDunbox);
    armPID.setOutputRange(ArmConstants.minOut, ArmConstants.maxOut);
    armPID.setP(ArmConstants.kPArm);
    armPID.setI(ArmConstants.kIArm);
    armPID.setD(ArmConstants.kDArm);
    armPID.setMaxI(ArmConstants.kMaxI);

  }

  public void moveToPos(double targetPos) {
    //armPID.setReference(targetPos, PIDposition);
    target = targetPos;
  }

  public void top() {
    //ArmEncoder.setPosition(ArmConstants.kTopPos); //units unknown
    armPID.setReference(ArmConstants.kTopPos, PIDposition);
  }

  public void middle() {
    //ArmEncoder.setPosition(ArmConstants.kMiddlePos); //units unknown
    armPID.setReference(ArmConstants.kMiddlePos, PIDposition);
  }

  public void stowed() {
    //ArmEncoder.setPosition(ArmConstants.kStowedPos); //units unknown
    armPID.setReference(ArmConstants.kStowedPos, PIDposition);
  }

  public void down() {
    armPID.setReference(ArmConstants.kDownPos, PIDposition);
  }

  public void shelf() {
    armPID.setReference(ArmConstants.kShelfPos, PIDposition);
  }

  public void moveUp() {
    ArmMotor.set(ArmConstants.maxOut * 0.5); //units unknown
  }

  public void moveDown() {
    ArmMotor.set(ArmConstants.minOut * 0.5); //units unknown
  }

  public void manualArm(double speed){
    if(Math.signum(speed) == -1){
      ArmMotor.set(speed * ArmConstants.minOut * 0.5);

    }
    if(Math.signum(speed) == 1){
      ArmMotor.set(speed * ArmConstants.maxOut * 0.5);

    }
  }

  public void stop() {
    ArmMotor.set(0.0);
  }

  @Override
  public void periodic() {
    if(setpoint < target){
      setpoint += 50;
    }
    if (setpoint > target){
      setpoint -=50;
    }
    armPID.setReference(setpoint, PIDposition);

    // This method will be called once per scheduler run
  }
} // end class
