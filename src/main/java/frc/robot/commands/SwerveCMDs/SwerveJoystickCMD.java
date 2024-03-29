package frc.robot.commands.SwerveCMDs;

import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        //Get Joystick Readings
        double xSpeed = xbox.getRawAxis(OIConstants.kDriverXAxis) * 0.875;
        double ySpeed = xbox.getRawAxis(OIConstants.kDriverYAxis) * 0.875;
        double turnSpeed = -xbox.getRawAxis(OIConstants.kDriverTurnAxis) * 0.875;

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
            // SmartDashboard.putNumber("Desired Heading ", swerveSubsystem.desiredHeading.getRadians());
        }

        fieldOriented = swerveSubsystem.getFieldOrient();

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
        if (fieldOriented) { //
            //Field Relative
            //DriverStation.reportWarning("getHeading ", false);
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turnSpeed,
                    swerveSubsystem.getRotation2D());

            if (chassisSpeeds.vxMetersPerSecond != 0 || chassisSpeeds.vyMetersPerSecond != 0) {
                chassisSpeeds = swerveSubsystem.fieldOrientedThetaHold(chassisSpeeds);
            }
        } else {
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turnSpeed);
        }

        //Making Module States
        SwerveModuleState[] moduleStates = swerveSubsystem.doIKMathSwerveModuleStates(chassisSpeeds);
        //Output to Wheels

        double swerveSpeed = (swerveSubsystem.slowed ? DriveConstants.kTeleDriveMaxSpeedMetersPerSecondSlow : DriveConstants.kTeleDriveMaxSpeedMetersPerSecond);

        for (int i = 0; i < 4; i++) {
            moduleStates[i].speedMetersPerSecond = moduleStates[i].speedMetersPerSecond
                    * swerveSpeed; //DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;

        }

        swerveSubsystem.setModuleStates(moduleStates);

        //SmartDashboard.putString("States", moduleStates.toString());  

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