package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.PID.PID;

public class SwerveSubsystem extends SubsystemBase {

    int iX = 0;

    private double previousAngle = 0;
    private double[] lastOutputAngle = new double[4];
    public Rotation2d desiredHeading = new Rotation2d(0);

    private boolean fieldOrient = true;
    public Pose2d odoPose = new Pose2d(new Translation2d(0, 0), new Rotation2d(0));

    PIDController headingPID = new PIDController(DriveConstants.kPTheta, DriveConstants.kITheta,
            DriveConstants.kDTheta);

    private final SlewRateLimiter[] speedLimiter = new SlewRateLimiter[4];
    private final SlewRateLimiter[] turnLimiter = new SlewRateLimiter[4];

    public final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDrivePort,
            DriveConstants.kFrontLeftAnglePort,
            DriveConstants.kFrontLeftDriveReversed,
            DriveConstants.kFrontLeftAngleReversed,
            DriveConstants.kFrontLeftAbsoluteEncoderPort,
            DriveConstants.kFrontLeftAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontLeftAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDrivePort,
            DriveConstants.kFrontRightAnglePort,
            DriveConstants.kFrontRightDriveReversed,
            DriveConstants.kFrontRightAngleReversed,
            DriveConstants.kFrontRightAbsoluteEncoderPort,
            DriveConstants.kFrontRightAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightAbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDrivePort,
            DriveConstants.kBackLeftAnglePort,
            DriveConstants.kBackLeftDriveReversed,
            DriveConstants.kBackLeftAngleReversed,
            DriveConstants.kBackLeftAbsoluteEncoderPort,
            DriveConstants.kBackLeftAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftAbsoluteEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDrivePort,
            DriveConstants.kBackRightAnglePort,
            DriveConstants.kBackRightDriveReversed,
            DriveConstants.kBackRightAngleReversed,
            DriveConstants.kBackRightAbsoluteEncoderPort,
            DriveConstants.kBackRightAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightAbsoluteEncoderReversed);

    private AHRS gyro = new AHRS(SPI.Port.kMXP);

    public final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics,
            new Rotation2d(0), new SwerveModulePosition[] {
                    frontLeft.getState(), frontRight.getState(),
                    backLeft.getState(), backRight.getState() },
            new Pose2d(
                    new Translation2d(0, 0),
                    new Rotation2d(0))); // Could Add OPTIONAL ROBOT Starting pose for field posing

    public SwerveSubsystem() {
        SmartDashboard.putNumber("P Gain", 0);
        SmartDashboard.putNumber("I Gain", 0);
        SmartDashboard.putNumber("D Gain", 0);
        SmartDashboard.putNumber("I Zone", 0);
        SmartDashboard.putNumber("Feed Forward", 0);
        SmartDashboard.putNumber("Max Output", 0);
        SmartDashboard.putNumber("Min Output", 0);
        SmartDashboard.putNumber("Set Rotations", 0);
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
        for (int i = 0; i < 4; i++) {
            speedLimiter[i] = new SlewRateLimiter(5);// TODO put in constants
            turnLimiter[i] = new SlewRateLimiter(30);
        }
        headingPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void zeroHeading() {
        gyro.reset();
        desiredHeading = new Rotation2d(0);
    }

    public double getHeading() {
        // DriverStation.reportWarning("getHeading " + gyro.getAngle(), false);
        return gyro.getAngle() * -1;
        // return Math.IEEEremainder(gyro.getAngle(), 360);

    }

    public Rotation2d getRotation2D() {

        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose2d() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        // SmartDashboard.putString("Front Left Reset Odometery DEBUG",
        // frontLeft.getState().toString());
        // SmartDashboard.putString("Front Left Reset Odometery DEBUG2",
        // pose.toString());
        odometer.resetPosition(new Rotation2d(getRotation2D().getRadians() - Math.PI / 2), new SwerveModulePosition[] {
                frontLeft.getState(), frontRight.getState(),
                backLeft.getState(), backRight.getState()
        }, pose);
    }

    @Override
    public void periodic() {
        PID.updatePeriods(frontLeft.angleEncoder.getPosition());
        odometer.update(getRotation2D(),
                new SwerveModulePosition[] {
                        frontLeft.getState(), frontRight.getState(),
                        backLeft.getState(), backRight.getState()
                });
        // Odometer will drift after dis and renenabling TODO: fix robot odometer
        // randomness after reenabling
        // SmartDashboard.putP
        odoPose = getPose2d();
        // DriverStation.reportWarning("Stored Odometry " +
        // odoPose.getTranslation().toString(), false);

        // SmartDashboard.putData(frontLeft.drivePIDController);
        // SmartDashboard.putNumber("Front Left Drive PID Error",
        // frontLeft.drivePIDController.getVelocityError());
        double pS = SmartDashboard.getNumber("P Gain A", 0);
        double iS = SmartDashboard.getNumber("I Gain A", 0);
        double dS = SmartDashboard.getNumber("D Gain A", 0);
        double izS = SmartDashboard.getNumber("I Zone A", 0);
        double ffS = SmartDashboard.getNumber("Feed Forward A", 0);
        double maxS = SmartDashboard.getNumber("Max Output A", 0);
        double minS = SmartDashboard.getNumber("Min Output A", 0);

        double pD = SmartDashboard.getNumber("P Gain D", 0);
        double iD = SmartDashboard.getNumber("I Gain D", 0);
        double dD = SmartDashboard.getNumber("D Gain D", 0);
        double izD = SmartDashboard.getNumber("I Zone D", 0);
        double ffD = SmartDashboard.getNumber("Feed Forward D", 0);
        double maxD = SmartDashboard.getNumber("Max Output D", 0);
        double minD = SmartDashboard.getNumber("Min Output D", 0);
        
        if (iX == 200){
            iX = 0;
            frontLeft.changePIDValued(pS, iS, dS, izS, ffS, maxS, minS, pD, iD, dD, izD, ffD, maxD, minD);
            frontRight.changePIDValued(pS, iS, dS, izS, ffS, maxS, minS, pD, iD, dD, izD, ffD, maxD, minD);
            backLeft.changePIDValued(pS, iS, dS, izS, ffS, maxS, minS, pD, iD, dD, izD, ffD, maxD, minD);
            backRight.changePIDValued(pS, iS, dS, izS, ffS, maxS, minS, pD, iD, dD, izD, ffD, maxD, minD);


            //DriverStation.reportWarning("------------------------------------------------------------------------------", false);
        } else {
            iX++;
        }

    }

    public void resetOdometryCache() {
        resetOdometry(odoPose);
        // DriverStation.reportWarning("Reset Odometry " +
        // odoPose.getTranslation().toString(), false);

    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void zeroAllModules() {
        frontLeft.resetEncoders();
        frontRight.resetEncoders();
        backLeft.resetEncoders();
        backRight.resetEncoders();
    }

    public void encoderPrintoutDeg() {
        // SmartDashboard.putNumber("Front Left Radians no Offset",
        // (frontLeft.getAbsoluteEncoderRadNoOffset()));
        // SmartDashboard.putNumber("Front Right Radians no Offset",
        // (frontRight.getAbsoluteEncoderRadNoOffset()));
        // SmartDashboard.putNumber("Back Left Encoder Radians no Offset",
        // (backLeft.getAbsoluteEncoderRadNoOffset()));
        // SmartDashboard.putNumber("Back Right Encoder Radians no Offset",
        // (backRight.getAbsoluteEncoderRadNoOffset()));
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);

        // SmartDashboard.putNumber("Front Left: ", frontLeft.getAbsoluteEncoderRad());
        // SmartDashboard.putNumber("Front Right: ",
        // frontRight.getAbsoluteEncoderRad());
        // SmartDashboard.putNumber("Back Left: ", backLeft.getAbsoluteEncoderRad());
        // SmartDashboard.putNumber("Back Right: ", backRight.getAbsoluteEncoderRad());

        // SmartDashboard.putString("Front Left State: ", "" + desiredStates[0]);
        // SmartDashboard.putString("Front Right State: ", "" + desiredStates[1]);
        // SmartDashboard.putString("Back Left State: ", "" + desiredStates[2]);
        // SmartDashboard.putString("Back Right State: ", "" + desiredStates[3]);
    }

    /**
     * Calculates the speed and angle of each wheel based on the linear and angular
     * speeds of the robot's chassis
     * 
     * @param chassisSpeeds the linear x, y, and angular speeds of the robot's
     *                      chassis
     * @return an array of SwerveModuleState objects, which contain the speed and
     *         angle for each wheel
     */
    public SwerveModuleState[] getIKMathSwerveModuleStates(ChassisSpeeds chassisSpeeds) {
        double x = chassisSpeeds.vxMetersPerSecond;
        double y = chassisSpeeds.vyMetersPerSecond;
        double theta = chassisSpeeds.omegaRadiansPerSecond;

        double[][] moduleCoord_in = new double[][] {
                { -DriveConstants.kTrackWidth, DriveConstants.kTrackWidth, -DriveConstants.kTrackWidth,
                        DriveConstants.kTrackWidth },
                { DriveConstants.kWheelBase, DriveConstants.kTrackWidth, -DriveConstants.kTrackWidth,
                        -DriveConstants.kTrackWidth } }; // TODO: move to constants: enumeration type ex: FrontLeft,
                                                         // FrontRight, BackLeft,BackRight
        SwerveModuleState[] outLinear = new SwerveModuleState[4]; // linearSpeeds
        SwerveModuleState[] outRotation = new SwerveModuleState[4]; // rotationSpeeds
        SwerveModuleState[] outSum = new SwerveModuleState[4]; // finalSpeeds
        // TODO: move this code somewhere else
        if (x == 0 && y == 0 && theta == 0) { // checks if all speeds are zero, if so returns an array of
                                              // SwerveModuleState objects with zero speed and the last output angle for
                                              // each wheel.
            return new SwerveModuleState[] {
                    new SwerveModuleState(0, new Rotation2d(lastOutputAngle[0])),
                    new SwerveModuleState(0, new Rotation2d(lastOutputAngle[1])),
                    new SwerveModuleState(0, new Rotation2d(lastOutputAngle[2])),
                    new SwerveModuleState(0, new Rotation2d(lastOutputAngle[3]))
            };
        }

        double linear_angle_component;

        if (x == 0 && y == 0) {
            linear_angle_component = previousAngle; // Defaulted to 0

        } else {
            linear_angle_component = Math.atan2(y, x);
            /*
             * returns the angle of the point (y, x) from the origin, with respect to the
             * positive x-axis
             * (calculates the angle between the x-axis going up and the specified x,y
             * cordinates)
             * ex: (4,3) returns 0.93 because the point is located at 0.93 radians
             * counterclockwise from the positive x-axis
             * Atan2 an handle the case when x = 0 and y = 0 correctly and return the
             * correct angle
             * particularly useful when working with points that have both positive and
             * negative coordinates because it can return the correct angle of the point
             * regardless of the signs of the x and y coordinates
             * ex: (-4, 3) returns -0.93 which corresponds to the angle of the point in the
             * third quadrant
             * ex: (3,-4) returns 2.49 which corresponds to the angle of the point in the
             * second quadrant
             */
            previousAngle = linear_angle_component;
        }

        double speed = Math.hypot(x, y);
        /*
         * calculates the speed of the wheel by finding the magnitude of the linear
         * component of the wheel
         * velocity is a vector quantity, which means it has both magnitude and
         * direction
         * the magnitude of the velocity vector of the wheel is the speed of the wheel
         * Math.sqrt(Math.pow(y, 2) + Math.pow(x, 2)); calculates the magnitude of the
         * velocity vector by using the Pythagorean theorem
         */
        // TODO: create helper method that iterates through the 4 wheels and creates a
        // SwerveModuleState object, and assigns it to an array element(lines: 196, 204,
        // 216)
        for (int i = 0; i < 4; i++) {// for each wheel, creates a new SwerveModuleState object with the previously
                                     // calculated speed and angle
                                     // calculates the angle and speed of the wheel based on the position of the
                                     // wheel and the angular speed of the robot
            outLinear[i] = new SwerveModuleState(speed,
                    new Rotation2d(linear_angle_component - DriveConstants.kHomingThetaRad));
            outRotation[i] = new SwerveModuleState(theta,
                    new Rotation2d(Math.atan2(moduleCoord_in[1][i], moduleCoord_in[0][i]) + Math.toRadians(90)
                            + DriveConstants.kHomingThetaRad));

            double vector1X = outLinear[i].speedMetersPerSecond * Math.cos(outLinear[i].angle.getRadians());
            double vector1Y = outLinear[i].speedMetersPerSecond * Math.sin(outLinear[i].angle.getRadians());
            double vector2X = outRotation[i].speedMetersPerSecond * Math.cos(outRotation[i].angle.getRadians());
            double vector2Y = outRotation[i].speedMetersPerSecond * Math.sin(outRotation[i].angle.getRadians());

            double sumX = vector1X + vector2X;
            double sumY = vector1Y + vector2Y;

            outSum[i] = new SwerveModuleState(
                    Math.sqrt(sumX * sumX + sumY * sumY),
                    new Rotation2d(Math.atan2(sumY, sumX)));
            lastOutputAngle[i] = outSum[i].angle.getRadians();
        } // Rotation2d object represents the angle of the linear component of the wheel
        /*
         * the for loop is iterating through each wheel and adding the linear and
         * rotation components of the wheel
         * the linear and rotation components are represented as polar vectors
         * polar vector: represented by an angle and a magnitude
         * normal vector: represented by an x and y component
         * 
         * vector1X, vector1Y etc. are converting the polar vectors to normal vectors by
         * multiplying the magnitude of the linear and rotation components by the cosine
         * and sine of their respective angles
         * you use the sine function to find the y component and the cosine function to
         * find the x component
         * sine: ratio of the side opposite of the angle to the hypotenuse of the
         * triangle
         * cosine: ratio of the adjacent side to the hypotenuse
         * both of these give the x and y components with that angle and a magnitude of
         * 1
         * so by multiplying the speed, you're effectively scaling the x and y
         * components of a vector with that angle and a magnitude of 1
         * 
         * after converting the polar vectors to normal vectors, it then adds the x and
         * y components of the linear and rotation vectors separately to get the x and y
         * components of the sum vector
         * 
         * finally, it converts the sum vector back to a polar vector representation
         * it uses the atan2 function to find the angle of the sum vector using the x
         * and y components, and uses the Pythagorean theorem to find the magnitude of
         * the sum vector
         * atan2 explained further below linear_angle_component = Math.atan2(y, x)
         *
         * then it creates a new SwerveModuleState object for the current wheel with the
         * speed and angle of the sum vector
         *
         * lastOutputAngle[i] = outSum[i].angle.getRadians();
         * this is to store the angle of the sum vector for each wheel in the
         * lastOutputAngle array, in case this function is called again with zero speed.
         *
         * https://en.wikipedia.org/wiki/Polar_coordinate_system
         * https://en.wikipedia.org/wiki/Cartesian_coordinate_system
         * https://en.wikipedia.org/wiki/Trigonometry
         * https://en.wikipedia.org/wiki/Sine
         * https://en.wikipedia.org/wiki/Cosine
         * https://en.wikipedia.org/wiki/Atan2
         * (research sources)
         */

        return outSum;
    }

    public void toggleFieldOrient() {
        fieldOrient = !fieldOrient;
    }

    public boolean getFieldOrient() {
        return fieldOrient;
    }

    public void setLastValidHeading(Rotation2d heading) {
        desiredHeading = heading;
    }

    public ChassisSpeeds fieldOrientedThetaHold(ChassisSpeeds chassisSpeeds) {
        Rotation2d robotHeading = getRotation2D();

        double inX = chassisSpeeds.vxMetersPerSecond;
        double inY = chassisSpeeds.vyMetersPerSecond;
        double inTheta = chassisSpeeds.omegaRadiansPerSecond;

        // double deltaTheta = Math.abs(desiredHeading.getRadians() -
        // robotHeading.getRadians());

        headingPID.setSetpoint(desiredHeading.getRadians());

        double thetaPIDCorrect = DriveConstants.kHeadingPIDMax * headingPID.calculate(robotHeading.getRadians());

        // SmartDashboard.putNumber("HEADING PID ", thetaPIDCorrect);

        inTheta += thetaPIDCorrect;

        return new ChassisSpeeds(inX, inY, inTheta);

    }

    public boolean isStopped() {
        double flSpeed = frontLeft.getDriveVelocity();
        double frSpeed = frontRight.getDriveVelocity();
        double blSpeed = backLeft.getDriveVelocity();
        double brSpeed = backRight.getDriveVelocity();

        // PID.setPID(frontLeft.anglePIDController);

        return Math.abs(flSpeed) < AutoConstants.kVelocityTolerance &&
                Math.abs(frSpeed) < AutoConstants.kVelocityTolerance &&
                Math.abs(blSpeed) < AutoConstants.kVelocityTolerance &&
                Math.abs(brSpeed) < AutoConstants.kVelocityTolerance;

    }
}