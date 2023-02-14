// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GrabberConstants;

public class GrabberSubsystem extends SubsystemBase {
  public TalonSRX handMotor;

  public GrabberSubsystem() {
    handMotor = new TalonSRX(GrabberConstants.kPortNumber);

    handMotor.selectProfileSlot(0, 0);

    handMotor.config_kP(0, GrabberConstants.kPhand);
    handMotor.config_kI(0, GrabberConstants.kIhand);
    handMotor.config_kD(0, GrabberConstants.kDhand);
  }

  public void closeHand() {
    handMotor.set(ControlMode.Position, GrabberConstants.kClosedPos);
  }

  public void openHand() {
    handMotor.set(ControlMode.Position, GrabberConstants.kOpenPos);
  }

  public void stopHand() {
    handMotor.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
