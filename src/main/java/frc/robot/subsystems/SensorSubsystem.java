// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.Serial;
import java.sql.Driver;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SensorSubsystem extends SubsystemBase {
  /** Creates a new SensorSubsystem. */
  SerialPort port;
  double distance;
  String read;

  public SensorSubsystem() {
    port = new SerialPort(9600, SerialPort.Port.kOnboard);
  }

  @Override
  public void periodic() {
    try{
      read = port.readString();
      // DriverStation.reportWarning(read, false);
    
      if(read != null){  
        read = read.replaceAll("R", "");
      
        String[] lines = read.split("\r");
        if (lines.length != 0){
          distance = Integer.parseInt(lines[0]);
        }

        SmartDashboard.putNumber("Sonic Distance", distance);
      }
    } catch(Exception e){
      DriverStation.reportWarning("It went badly " + e, false);
    }
    // This method will be called once per scheduler run
  }
}
