package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;

//import java.util.function.Supplier;

//import edu.wpi.first.math.filter.SlewRateLimiter;
//import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystickCMD extends CommandBase {
    XboxController xbox = new XboxController(1);
    private final SwerveSubsystem swerveSubsystem; //MAYBE THESE NEED FINAL
    private boolean fieldOriented;
    //private final SlewRateLimiter xLimiter, yLimiter, turnLimiter;

    public SwerveJoystickCMD(SwerveSubsystem swerveSubsystem) {

        this.swerveSubsystem = swerveSubsystem;

        this.fieldOriented = swerveSubsystem.getFieldOrient();
        //this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        //this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        //this.turnLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        //Get Joystick Readings
        // double xSpeed = xSpdFunction.get();
        // double ySpeed = ySpdFunction.get();
        // double turnSpeed = turnSpdFunction.get();
        double xSpeed = xbox.getRawAxis(OIConstants.kDriverXAxis);
        double ySpeed = xbox.getRawAxis(OIConstants.kDriverYAxis);
        double turnSpeed = xbox.getRawAxis(OIConstants.kDriverTurnAxis);

        xSpeed *= -1; //invert the direction of the robot movement on the x-axis.

        //Deadbanding
        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
        turnSpeed = Math.abs(turnSpeed) > OIConstants.kDeadband ? turnSpeed : 0.0;

        double headingRadians = swerveSubsystem.getRotation2D().getRadians();
        if (turnSpeed != 0) {
            if (headingRadians > Math.PI) {
                headingRadians -= 2 * Math.PI;
            }
            if (headingRadians < -Math.PI) {
                headingRadians += 2 * Math.PI;
            }

            swerveSubsystem.setLastValidHeading(new Rotation2d(headingRadians));
            SmartDashboard.putNumber("Desired Heading ", swerveSubsystem.desiredHeading.getRadians());
        }
        //DriverStation.reportWarning("Spood: " + turnSpeed, false);
        //Ratelimiting
        //Moved to function in the Subsystem and done after IK
        //DriverStation.reportWarning("Speed: " + turnSpeed, false);

        fieldOriented = swerveSubsystem.getFieldOrient();

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
        if (fieldOriented) { //
            //Field Relative
            //DriverStation.reportWarning("getHeading ", false);
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turnSpeed,
                    swerveSubsystem.getRotation2D());

            chassisSpeeds = swerveSubsystem.fieldOrientedThetaHold(chassisSpeeds);
        } else {
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turnSpeed);
        }
        //Making Module States
        //SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveModuleState[] moduleStates = swerveSubsystem.getIKMathSwerveModuleStates(chassisSpeeds);
        //Output to Wheels

        //moduleStates = swerveSubsystem.rateLimitModuleStates(moduleStates);

        for (int i = 0; i < 4; i++) {
            moduleStates[i].speedMetersPerSecond = moduleStates[i].speedMetersPerSecond
                    * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;

        }

        swerveSubsystem.setModuleStates(moduleStates);

    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}