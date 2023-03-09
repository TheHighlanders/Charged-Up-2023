// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GrabberConstants;
import frc.robot.utilities.CANSparkMaxCurrent;

public class GrabberSubsystem extends SubsystemBase {
  public CANSparkMaxCurrent handMotor;

  public GrabberSubsystem() {
    handMotor = new CANSparkMaxCurrent(GrabberConstants.kPortNumber, MotorType.kBrushless);
    handMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
  }

  public void closeHand() {
    handMotor.setCurrent(GrabberConstants.kClosingAmps);
  }

  public void openHand() {
    handMotor.setCurrent(GrabberConstants.kOpeningAmps);
  }

  public void stopHand() {
    handMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
