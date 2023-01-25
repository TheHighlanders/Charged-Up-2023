// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GrabberConstants;
import frc.robot.commands.GrabberCMDs.GrabberCloseCMD;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Compressor;


public class GrabberSubsystem extends SubsystemBase {
  public DoubleSolenoid squishgrabber = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
  public Compressor airtank = new Compressor(0, PneumaticsModuleType.CTREPCM);
  public GrabberSubsystem() {


  }
public void Closed(){
  squishgrabber.set(Value.kReverse);
}
public void Open(){
  squishgrabber.set(Value.kForward);
}
public void Off(){
  squishgrabber.set(Value.kOff);
}

public void Start(){
  airtank.enableDigital();
}

public void Stop(){
  airtank.disable();
}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
