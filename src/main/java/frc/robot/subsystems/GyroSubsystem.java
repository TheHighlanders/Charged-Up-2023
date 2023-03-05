// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GyroConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.kauailabs.navx.frc.AHRS;
//import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class GyroSubsystem extends SubsystemBase {
  //PIDController VpidController = new PIDController(GyroConstants.Vkp, GyroConstants.Vki, GyroConstants.Vkd);
  //PIDController TpidController = new PIDController(GyroConstants.Tkp, GyroConstants.Tki, GyroConstants.Tkd);
  AHRS ahrs;
  /** Creates a new GyroSubsystem. */
  public GyroSubsystem() {
    ahrs = new AHRS();
    ahrs.calibrate();
  }

  public SwerveModuleState[] CalculateAngle(SwerveSubsystem subsystem) {
    float roll = ahrs.getRoll() * -1; // it was the wrong direction
    float pitch = ahrs.getPitch()* -1;
    double angle = Math.atan2(pitch, roll);
    double magnitue = clamp(Math.hypot(pitch, roll));
    //double Vpid = VpidController.calculate((yaw + pitch)/2, 0);
    // double xSpeed =  magnitue * Math.cos(angle);
    // double ySpeed = magnitue * Math.sin(angle);
    SmartDashboard.putString("Gyro values", "pitch/yaw " + roll + " " + pitch);
    SwerveModuleState state = new SwerveModuleState(magnitue, new Rotation2d(angle- Math.PI/2));
    SmartDashboard.putString("Module state", state.toString());
    return new SwerveModuleState[] {state, state, state, state};

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
}
