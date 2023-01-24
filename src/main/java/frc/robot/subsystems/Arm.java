// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.ArmConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  public CANSparkMax ArmMotor;
  public RelativeEncoder ArmEncoder;

  /** Creates a new Arm. */
  public Arm() {
    ArmMotor = new CANSparkMax ( ArmConstants.ARM_MOTOR_ID, MotorType.kBrushless );
    ArmEncoder = ArmMotor.getAlternateEncoder ( ArmConstants.COUNTS_PER_REV );
  }

  public void top() {
    ArmEncoder.setPosition(ArmConstants.kTop); //units unknown
  }

  public void middle() { 
    ArmEncoder.setPosition(ArmConstants.kMiddle); //units unknown
  }

  public void stowed() {
    ArmEncoder.setPosition(ArmConstants.kStowed); //units unknown
  }

  public void up() {
    ArmMotor.set(1.0); //units unknown
  }

  public void down() { 
    ArmMotor.set(-1.0); //units unknown
  }

  public void stop() {
    ArmMotor.set(0.0);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
} // end class
