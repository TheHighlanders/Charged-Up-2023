// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonCMDs;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class AUTOcsvPathFollowCMD extends CommandBase {
  /** Creates a new AUTOcsvPathFollowCMD. */
  private BufferedReader reader;
  private int closestPointIndex = 0;
  private SwerveSubsystem swerveSubsystem;

  private PIDController pidController = new PIDController(AutoConstants.kXPIDp, AutoConstants.kXPIDi, AutoConstants.kXPIDd);

  private double[] xArray;
  private double[] yArray;
  private double[] headingArray;
  private double[] timeArray;
  private boolean cmdDone = false;
  private Rotation2d gyroOffset;

  /**
   * Reads A CSV with the filename path, and parses it out into Arrays for use in
   * following
   * 
   * @param path             The filename of the target path
   * @param swerve_subsystem The Swerve Drive
   */
  public AUTOcsvPathFollowCMD(String path, SwerveSubsystem swerve_subsystem) {
    swerveSubsystem = swerve_subsystem;
    addRequirements(swerveSubsystem);
    
    ArrayList<Double> xList = new ArrayList<>();
    ArrayList<Double> yList = new ArrayList<>();
    ArrayList<Double> timeList = new ArrayList<>();
    ArrayList<Double> headingList = new ArrayList<>();

    try {
      reader = new BufferedReader(new FileReader(path));
      reader.readLine();
      reader.readLine();
      String line = reader.readLine();

      while (line != null) {
        // System.out.println(line);
        // read next line

        String[] values = line.split(",");
        double xPos = Double.parseDouble(values[1]);
        double yPos = Double.parseDouble(values[2]);
        double time = Double.parseDouble(values[0]);
        double heading = Double.parseDouble(values[7]);

        xList.add(xPos);
        yList.add(yPos);
        headingList.add(heading);
        timeList.add(time);

        line = reader.readLine();
      }
      reader.close();
    } catch (IOException e) {
      e.printStackTrace();
    }

    xArray = new double[xList.size()];
    yArray = new double[yList.size()];
    headingArray = new double[headingList.size()];
    timeArray = new double[timeList.size()];

    for (int i = 0; i < xList.size(); i++) {
      xArray[i] = xList.get(i).doubleValue();
      yArray[i] = yList.get(i).doubleValue();
      headingArray[i] = headingList.get(i).doubleValue();
      timeArray[i] = timeList.get(i).doubleValue();
    }
    //SmartDashboard.putNumber("Last X", xArray[xArray.length - 1]);
    //SmartDashboard.putNumber("Last Y", yArray[yArray.length - 1]);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    boolean mirrorField = (DriverStation.getAlliance() == DriverStation.Alliance.Red);

    if(mirrorField){
      for (int i = 0; i < xArray.length; i++) {
        xArray[i] = AutoConstants.fieldLength - xArray[i];
        yArray[i] = AutoConstants.fieldLength - yArray[i];
        headingArray[i] = headingArray[i] * -1 + AutoConstants.headingFlipMirror;
      }
    }//Maybe x And y are sawpperd,(addressde in chassisSpeeds) and check gyro/odo resetting stuff
    Pose2d pathStartPose = new Pose2d(new Translation2d(xArray[0], yArray[0]), new Rotation2d(edu.wpi.first.math.MathUtil.angleModulus(Units.degreesToRadians(headingArray[0] + 90))));
    //swerveSubsystem.zeroHeading(); ?? Trying fix 4:03pm
    gyroOffset = swerveSubsystem.getRotation2D();
    swerveSubsystem.resetOdometry(pathStartPose);
    cmdDone = false;
    closestPointIndex = 0;

  }

  // Called every time the scheduler runs while the command is scheduled.

  /**
   * The drive to point code from AUTOswerveMoveCommand, but adapted to pick the
   * target point
   * using a 3d Pythagorean distance search, crazySearch()
   * 
   * The target point is then changed to be the point 1 second away,
   * or the end of the path, if we are less than 1 second to the end
   */
  @Override
  public void execute() {

    Pose2d currentPose = swerveSubsystem.getPose2d();

    double currentX = currentPose.getX();
    double currentY = currentPose.getY();

    closestPointIndex = crazySearch(currentX, currentY);
    int targetPointIndex = distanceDelta(closestPointIndex, currentX, currentY);
     DriverStation.reportWarning(
         "Closest Point Index: " + closestPointIndex + "\nTarget Point Index: " + targetPointIndex, false);
    double xEndPoint = xArray[targetPointIndex];
    double yEndPoint = yArray[targetPointIndex];
    Rotation2d headingEndPoint = new Rotation2d(Math.toRadians(headingArray[targetPointIndex]));

    // SmartDashboard.putNumber("X Target", xEndPoint);
    // SmartDashboard.putNumber("Y Target", yEndPoint);
    // SmartDashboard.putNumber("theta Target", headingEndPoint.getRadians());

    double deltaX = currentX - xEndPoint;
    double deltaY = yEndPoint - currentY;

    

    double pid = Math.abs(pidController.calculate(Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2)), 0))
        * AutoConstants.kMaxSpeedMetersPerSecond;


    double speedX = (deltaX / (Math.abs(deltaX) + Math.abs(deltaY))) * pid;
    double speedY = (deltaY / (Math.abs(deltaX) + Math.abs(deltaY))) * pid;

    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        (Math.abs(deltaY) > AutoConstants.kTranslatePointError ? speedY : 0.0),
        (Math.abs(deltaX) > AutoConstants.kTranslatePointError ? speedX : 0.0),
        0.0,
        swerveSubsystem.getRotation2D().minus(gyroOffset));

    swerveSubsystem.setLastValidHeading(headingEndPoint);
    chassisSpeeds = swerveSubsystem.fieldOrientedThetaHold(chassisSpeeds);

    SwerveModuleState[] moduleStates = swerveSubsystem.doIKMathSwerveModuleStates(chassisSpeeds);

    swerveSubsystem.setModuleStates(moduleStates);
     DriverStation.reportWarning(
       "asdadClosest Point Index: " + closestPointIndex + "\nasdTarget Point Index: " + targetPointIndex, false);
  
    if (closestPointIndex == targetPointIndex) {
      cmdDone = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    DriverStation.reportWarning("DONE WITH TRAJECTORY" + cmdDone, false);
    return cmdDone;
  }

  /**
   * 3 Dimensional Pythagorean Theorem used to find the next closest point to the
   * robot, in both time and space
   * Search will stop if time is too big
   * 
   * @param currentX The current X position of the robot
   * @param currentY The current Y position of the robot
   * @return The index of the closest point
   */
  public int crazySearch(double currentX, double currentY) {
    double minZ = Double.POSITIVE_INFINITY;
    int out = closestPointIndex;
    for (int i = closestPointIndex; i < xArray.length; i++) {
      double search = Math.sqrt(
          Math.pow(currentX - xArray[i], 2) +
              Math.pow(currentY - yArray[i], 2));
      if (search < minZ) {
        minZ = search;
        out = i;
      }
      if (minZ < search) {
        // DriverStation.reportWarning("Crazy Break: " + i, false);
        break;
      }
    }
    return out;
  }

  /**
   * Takes the target point and advances it by 1 second, or sets it to the last
   * position, if we are close to the end
   * 
   * @param currentIndex The index of the closest point
   * @return The index of the point after the time translation
   */
  public int distanceDelta(int currentIndex, double currentX, double currentY) {
    for (int i = currentIndex; i < xArray.length - 1; i++) {
      double distance = Math.sqrt(Math.pow(xArray[i] - currentX, 2) + Math.pow(yArray[i] - currentY, 2));
      if (distance > 0.2) {
        //if(i == closestPointIndex && i < xArray.length-1){i++;}//Nye 3/8 may cause zooom
        return i;
      }
    }
    return timeArray.length - 1;
  }
}
