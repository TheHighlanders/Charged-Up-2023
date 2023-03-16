// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GyroConstants;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.kauailabs.navx.frc.AHRS;
//import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.MathUtil;

public class GyroSubsystem extends SubsystemBase {
  //PIDController VpidController = new PIDController(GyroConstants.Vkp, GyroConstants.Vki, GyroConstants.Vkd);
  //PIDController TpidController = new PIDController(GyroConstants.Tkp, GyroConstants.Tki, GyroConstants.Tkd);
  AHRS ahrs;
  public boolean balanced = false;
  public float roll;
  public float pitch;
  public double angle;
  public double balCount;
  /** Creates a new GyroSubsystem. */
  public GyroSubsystem() {
    ahrs = new AHRS();
    ahrs.calibrate();
    balanced = false;
    balCount = 0;
  }

  public SwerveModuleState[] CalculateAngle() {
    roll = ahrs.getRoll() * -1; // it was the wrong direction
    pitch = ahrs.getPitch()* -1;
    angle = Math.atan2(pitch, roll);
    //double magnitue = clamp(Math.hypot(pitch, roll));

    double magnitude = bangBang(Units.degreesToRadians(Math.hypot(roll, pitch)));
    //double Vpid = VpidController.calculate((yaw + pitch)/2, 0);
    // double xSpeed =  magnitue * Math.cos(angle);
    // double ySpeed = magnitue * Math.sin(angle);
    // SmartDashboard.putString("Gyro values", "pitch/yaw " + roll + " " + pitch);
    SwerveModuleState state = new SwerveModuleState(magnitude, new Rotation2d(angle- Math.PI/2));
    // SmartDashboard.putString("Module state", state.toString());
    return new SwerveModuleState[] {state, state, state, state};

  }

  public double getAngleDeg(){
    return pitch;
  }

  public double bangBang(double angleRobotRadians){
    // SmartDashboard.putNumber("gyroX", Units.radiansToDegrees(angleRobotRadians));
    double out = 0;
    double threshold = GyroConstants.kThreshold;
    // SmartDashboard.putNumber("Threshold", Units.radiansToDegrees(threshold));
    if(Math.abs(angleRobotRadians) >= threshold){
      out = GyroConstants.max;
    } else if (Math.abs(angleRobotRadians) >= Units.degreesToRadians(2.5)){
      out = GyroConstants.max * 0.9;
    }


    // SmartDashboard.putNumber("GyroOut", out);
    return out;
  }

  public double clamp(double x){
    if (x < GyroConstants.startInreaseTrend){
      return 0;
    }
    else if (x <= GyroConstants.stopInreaseTrend && x >= GyroConstants.startInreaseTrend){
      return (-(x-GyroConstants.startInreaseTrend)/GyroConstants.stopInreaseTrend * -GyroConstants.max);
    }
    else if (x > 15){
      return GyroConstants.max;
    }
    return 0;
  }
  
  public void periodic(){
    if(Math.abs(roll) <= 2 && Math.abs(pitch) <= 2){
      balCount++;
      // SmartDashboard.putString("State", "Balancing");
    }else{
      balCount = 0;
      // SmartDashboard.putString("State", "Reseting");
    }

    if(balCount >= 25){
      balanced = true;
    } else {
      balanced = false;
    }
    // SmartDashboard.putNumber("Test", Units.degreesToRadians(15));
  }
}
