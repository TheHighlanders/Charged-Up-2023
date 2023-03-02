package frc.robot.subsystems;

// BASED ON https://www.youtube.com/watch?v=0Xi9yb1IMyA
//import com.revrobotics.AnalogInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
//import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
//import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
//import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {

  private final CANSparkMax driveMotor;
  private final CANSparkMax angleMotor;

  private final RelativeEncoder driveEncoder;
  public final RelativeEncoder angleEncoder;

  private final CANSparkMax.IdleMode holdMode = IdleMode.kBrake;

  public final PIDController drivePIDController;
  public final PIDController anglePIDController;

  public final edu.wpi.first.wpilibj.AnalogInput absoluteEncoder;
  private final boolean absoluteEncoderReversed;
  private final double absoluteEncoderOffsetRad;

  public SwerveModule(int driveMotorID, int angleMotorID, boolean driveMotorReversed, boolean angleMotorReversed,
      int absoluteEncoderID, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

    this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
    this.absoluteEncoderReversed = absoluteEncoderReversed;

    absoluteEncoder = new edu.wpi.first.wpilibj.AnalogInput(absoluteEncoderID);

    driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
    angleMotor = new CANSparkMax(angleMotorID, MotorType.kBrushless);

    driveMotor.setInverted(driveMotorReversed);
    angleMotor.setInverted(angleMotorReversed);

    driveEncoder = driveMotor.getEncoder();
    angleEncoder = angleMotor.getEncoder();

    driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveMotorEncoderRot2Meter);
    driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveMotorEncoderRPM2MeterPerSec);
    angleEncoder.setPositionConversionFactor(ModuleConstants.kAngleMotorEncoderRot2Rad);
    angleEncoder.setVelocityConversionFactor(ModuleConstants.kAngleMotorEncoderRPM2RadPerSec);

    driveMotor.setIdleMode(holdMode);

    anglePIDController = new PIDController(ModuleConstants.kPAngle, ModuleConstants.kIAngle, ModuleConstants.kDAngle);
    anglePIDController.enableContinuousInput(-Math.PI, Math.PI);

    drivePIDController = new PIDController(ModuleConstants.kPDrive, ModuleConstants.kIDrive, ModuleConstants.kDDrive);

    anglePIDController.setTolerance(ModuleConstants.kAngleTolerance);

    angleMotor.getForwardLimitSwitch(Type.kNormallyClosed).enableLimitSwitch(false); //disables the limit switches for the drive and angle motors
    angleMotor.getReverseLimitSwitch(Type.kNormallyClosed).enableLimitSwitch(false);

    driveMotor.getForwardLimitSwitch(Type.kNormallyClosed).enableLimitSwitch(false);
    driveMotor.getReverseLimitSwitch(Type.kNormallyClosed).enableLimitSwitch(false);

  }

  public double getDrivePosition() {
    return driveEncoder.getPosition();
  }

  public double getAnglePosition() {
    return angleEncoder.getPosition();
  }

  public double getDriveVelocity() {
    return driveEncoder.getVelocity();
  }

  public double getAngleVelocity() {
    return angleEncoder.getVelocity();
  }

  public double getAbsoluteEncoderRad() {
    double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
    angle *= 2.0 * Math.PI;
    angle -= absoluteEncoderOffsetRad;
    return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
  }

  public double getAbsoluteEncoderRadNoOffset() {
    double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
    angle *= 2.0 * Math.PI;
    return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
  }

  public void resetEncoders() {
    // driveEncoder.setPosition(0);
    angleEncoder.setPosition(getAbsoluteEncoderRad());
  }

  public SwerveModulePosition getState() {
    return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getAnglePosition()));
  }

  public static SwerveModuleState optimize(
      SwerveModuleState desiredState, Rotation2d currentAngle) {
    var delta = desiredState.angle.minus(currentAngle);

    if (Math.abs(delta.getDegrees()) > 90.0) {
      //DriverStation.reportWarning("Optizme Trigged", false);
      return new SwerveModuleState(
          -desiredState.speedMetersPerSecond,
          desiredState.angle.rotateBy(Rotation2d.fromDegrees(180.0)));
    } else {
      return new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
    }
  }

  public void setDesiredState(SwerveModuleState state) {

    state = optimize(state, getState().angle);

    driveMotor.set(drivePIDController.calculate(getDriveVelocity(), state.speedMetersPerSecond));

    angleMotor.set(anglePIDController.calculate(getAnglePosition(), state.angle.getRadians()));

  }

  public void stop() {
    driveMotor.set(0);
  }

  public void jog(double angleSpeed, double driveSpeed){
    DriverStation.reportWarning("Jogging: " + absoluteEncoder.getChannel(), false);

    angleMotor.set(angleSpeed);
    driveMotor.set(driveSpeed);

  }
}