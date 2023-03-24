// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GrabberConstants;
import frc.robot.utilities.CANSparkMaxCurrent;

public class GrabberSubsystem extends SubsystemBase {
  public CANSparkMaxCurrent handMotor;
  public RelativeEncoder handEncoder;
  public SparkMaxPIDController handPID;
  public boolean opening;

  public GrabberSubsystem() {
    handMotor = new CANSparkMaxCurrent(GrabberConstants.kPortNumber, MotorType.kBrushless);
    handMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    handEncoder = handMotor.getEncoder();
    handPID = handMotor.getPIDController();
    handPID.setOutputRange(-0.375, 0.375);
    handPID.setP(GrabberConstants.kPhand);
  }

  public void closeHand() {
    handMotor.setCurrent(GrabberConstants.kClosingAmps);
    opening = false;
  }

  public void openHand() {
    handMotor.setCurrent(GrabberConstants.kOpeningAmps);
    opening = true;
  }
  public void openHandPartial(double pos){
   handPID.setReference(pos, ControlType.kPosition);
  }
  public void stopHand() {
    handMotor.set(0);
  }


  public void zero(){
    handEncoder.setPosition(0);
    //DriverStation.reportWarning("POS" + handEncoder.getPosition(), false);
  }

  @Override
  public void periodic() {
    if(handMotor.getOutputCurrent() >= GrabberConstants.kClosingAmps){
      handPID.setReference(handEncoder.getPosition(), ControlType.kPosition);
    }
    // This method will be called once per scheduler run
  }
}
