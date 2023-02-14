// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class VisionConstants {
        public static final double kVisionHorizontalScalar = 1.0;

        public static final double kRedParkAngle = Math.PI / 2.0;
        public static final double kBlueParkAngle = Math.PI / -2.0;
    }

    public static final class IntakeConstants {
        public static final int INTAKE_MOTOR_ID = 19;
        public static final int INTAKE_DEPLOY_ID = 20;

        public static final int DEPLOY_PID_ID = 0;

        public static final double kPIntakeDeploy = 0;
        public static final double kIIntakeDeploy = 0;
        public static final double kDIntakeDeploy = 0;

        public static final double kIntakeDeployRatio = 0;

        public static final double kIntakeOutPos = 0;
        public static final double kIntakeInPos = 0;

        public static final double kIntakeSpeed = 1;

    }

    public static final class GrabberConstants {
        public static final double DoubleSolenoid = 1;
    }

    public static final class ArmConstants {

        public static final double kIntakeDeathZone = 1000;
        static final double gearboxRatio = 45.0 / 1.0;
        static final double beltRatio = 2.0 / 1.0; //GUESTIMATED find actual belt ratio from mechanical
        public static final double kArmRatio = gearboxRatio * beltRatio;

        public static final int ARM_MOTOR_ID = 62;

        public static final double kStowedPos = 750;
        public static final double kDownPos = 1500;
        public static final double kMiddlePos = 2000;//0.8
        public static final double kShelfPos = 2500;
        public static final double kTopPos = 2750;

        public static final double kPArm = 0.01;
        public static final double kIArm = 0;
        public static final double kDArm = 0;

        public static final int COUNTS_PER_REV = 0;
    }

    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 1.0f / 8.14; // was 5.8462, not sure why, changed to SDS L1 ratio
        public static final double kAngleMotorGearRatio = 1.0f / 12.8f; // Reciprocalized 1/12
        public static final double kDriveMotorEncoderRot2Meter = kDriveMotorGearRatio * Math.PI
                * kWheelDiameterMeters;
        public static final double kAngleMotorEncoderRot2Rad = kAngleMotorGearRatio * 2.0f * Math.PI;
        public static final double kDriveMotorEncoderRPM2MeterPerSec = kDriveMotorEncoderRot2Meter / 60.0f;
        public static final double kAngleMotorEncoderRPM2RadPerSec = kAngleMotorEncoderRot2Rad / 60.0f;
        public static final double kPAngle = 0.1;
        public static final double kIAngle = 0;
        public static final double kDAngle = 0;

        // public static final double kPAngle = 0.5;
        // public static final double kIAngle = 1.0;
        // public static final double kDAngle = 0.0;

        public static final double kAngleTolerance = 0.002;

        //Sketchy Velocity PID loop no use D pls TODO: Recalibrate for new correct GearRatio
        public static final double kPDrive = 0.1; //D? Dont do a Negative
        public static final double kIDrive = 0.75; //P? //Was 0.75 before gear ratio change, lowering
        public static final double kDDrive = 0; //??

        public static final SparkMaxLimitSwitch.Type kEncoderFakeLimitType = Type.kNormallyClosed;
        public static final double kAngleHomingSpeed = 0.25;
    }

    public static final class DriveConstants {

        public static final double kPTheta = 2.5; //10 with Ballast
        public static final double kITheta = 0; //0 with Ballast
        public static final double kDTheta = 0; //0 With Ballast

        public static final double kHeadingPIDMax = 0.2;

        //Robot Physical Parameters        
        //Speed Stats
        public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 6;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond
                / 4;

        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;

        //Size Info
        public static final double kTrackWidth = Units.inchesToMeters(23.5);
        public static final double kWheelBase = Units.inchesToMeters(23.5);
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

        //Module Info
        public static final double kWheelOffsetFrontToSideways = Math.toRadians(90);
        public static final double kHomingThetaRad = Math.toRadians(90);
        //Front Left
        public static final int kFrontLeftDrivePort = 2;
        public static final int kFrontLeftAnglePort = 4;
        public static final boolean kFrontLeftDriveReversed = false;
        public static final boolean kFrontLeftAngleReversed = false;
        public static final int kFrontLeftAbsoluteEncoderPort = 0;
        public static final double kFrontLeftAbsoluteEncoderOffsetRad = 5.519947;// + kWheelOffsetFrontToSideways;
        public static final boolean kFrontLeftAbsoluteEncoderReversed = true;

        //Front Right
        public static final int kFrontRightDrivePort = 7;
        public static final int kFrontRightAnglePort = 8;
        public static final boolean kFrontRightDriveReversed = false;
        public static final boolean kFrontRightAngleReversed = false;
        public static final int kFrontRightAbsoluteEncoderPort = 3;
        public static final double kFrontRightAbsoluteEncoderOffsetRad = 1.861201;// + kWheelOffsetFrontToSideways;
        public static final boolean kFrontRightAbsoluteEncoderReversed = true;

        //Back Left
        public static final int kBackLeftDrivePort = 3;
        public static final int kBackLeftAnglePort = 1;
        public static final boolean kBackLeftDriveReversed = false;
        public static final boolean kBackLeftAngleReversed = false;
        public static final int kBackLeftAbsoluteEncoderPort = 1;
        public static final double kBackLeftAbsoluteEncoderOffsetRad = 5.371961;// + kWheelOffsetFrontToSideways;
        public static final boolean kBackLeftAbsoluteEncoderReversed = true;

        //Back Right
        public static final int kBackRightDrivePort = 6;
        public static final int kBackRightAnglePort = 5;
        public static final boolean kBackRightDriveReversed = false;
        public static final boolean kBackRightAngleReversed = false;
        public static final int kBackRightAbsoluteEncoderPort = 2;
        public static final double kBackRightAbsoluteEncoderOffsetRad = 5.991324;// + kWheelOffsetFrontToSideways;
        public static final boolean kBackRightAbsoluteEncoderReversed = true;
    }

    public static final class OIConstants {
        public static final double kDeadband = 0.05;
        public static final int kdriverJoystick = 1;
        public static final int koperatorJoystick = 0;

        public static final int kDriverXAxis = 0;
        public static final int kDriverYAxis = 1;
        public static final int kDriverTurnAxis = 4;
        public static final int kDriverFieldOrientButton = 3;
    }

    public static final class AutoConstants {

        public static final double fieldCenterMeters = 8.270875;
        public static final double fieldLength = fieldCenterMeters * 2;
        public static final double headingFlipMirror = 90;

        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 24;
        public static final double kMaxAngularSpeedRadiansPerSecond = //
                DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;

        public static final double kTrajectoryPointSeperation = 0.2;

        public static final double kAprilTagParkingDistance = 1.0;

        public static final double kVelocityTolerance = 0.1;

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
        public static final TrapezoidProfile.Constraints kPThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond,
                kMaxAngularAccelerationRadiansPerSecondSquared);

        public static final double kTranslatePointError = 0.01;
        public static final double kTranslateSpeed = 0.5;

        public static final double kRotationError = 0.1;
        public static final double kRotationSpeed = 0.4;

        public static final double kXPIDp = 2.8;
        public static final double kXPIDi = 0;
        public static final double kXPIDd = 0.1;

    }

}
