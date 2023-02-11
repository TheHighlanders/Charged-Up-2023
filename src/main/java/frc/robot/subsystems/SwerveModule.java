package frc.robot.subsystems;

// BASED ON https://www.youtube.com/watch?v=0Xi9yb1IMyA
//import com.revrobotics.AnalogInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
//import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;

//import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
//import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {

  private final CANSparkMax driveMotor;
  private final CANSparkMax angleMotor;

  private final RelativeEncoder driveEncoder;
  public final RelativeEncoder angleEncoder;

  private final CANSparkMax.IdleMode holdMode = IdleMode.kBrake;

  public final SparkMaxPIDController drivePIDController;
  public final SparkMaxPIDController anglePIDController;

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

    anglePIDController = angleMotor.getPIDController();
    // anglePIDController.setP(ModuleConstants.kPAngle);
    // anglePIDController.setI(ModuleConstants.kIAngle);
    // anglePIDController.setD(ModuleConstants.kDAngle);
    // anglePIDController.setIZone(0);
    // anglePIDController.setFF(0);
    // anglePIDController.setOutputRange(-Math.PI, Math.PI);
    //anglePIDController.enableContinuousInput(-Math.PI, Math.PI);

    drivePIDController = driveMotor.getPIDController();
    // drivePIDController.setP(ModuleConstants.kPDrive);
    // drivePIDController.setI(ModuleConstants.kIDrive);
    // drivePIDController.setD(ModuleConstants.kDDrive);
    // drivePIDController.setIZone(0);
    // drivePIDController.setFF(0);
    // drivePIDController.setOutputRange(-Math.PI, Math.PI);
    //drivePIDController.calculate(absoluteEncoderID, absoluteEncoderOffset);
    //DriverStation.reportWarning("MODULE TOLERANCE " + anglePIDController.getPositionTolerance(), false);
    //TODO: Instead of hardcoding Type.kNormallyClosed for the limit switches, create a enumeration type to represent the different types of limit switches, such as NormallyClosed, NormallyOpen
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

    // boolean useDashboard = false;//SmartDashboard.getBoolean("Setpoint Control Enabled", false);
    // if (useDashboard) {
    //   angleMotor.set(anglePIDController.calculate(
    //       getAnglePosition(),
    //       //SmartDashboard.getNumber("Funny Setpoint Control", 0)));
    //   return;
    // }
    //DriverStation.reportWarning("Module.setDesiredState()", false);
    state = optimize(state, getState().angle);
    //SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] state", state.toString());

    //driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    drivePIDController.setReference(state.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);

    anglePIDController.setReference(state.angle.getRadians(), CANSparkMax.ControlType.kPosition);
    //SmartDashboard.putNumber("Module " + absoluteEncoder.getChannel() + " PID error: ",
        // anglePIDController.getPositionError());
    // SmartDashboard.putNumber("Module " + absoluteEncoder.getChannel(), anglePIDController.getSetpoint());

    // SmartDashboard.putNumber("Module " + absoluteEncoder.getChannel() + " Motor Encoder", getAnglePosition());
  }

  public void stop() {
    driveMotor.set(0);
  }

  public void changePIDValued(double pS, double iS, double dS, double izS, double ffS, double maxS, double minS, double pD, double iD, double dD, double izD, double ffD, double maxD, double minD){
    drivePIDController.setP(pS);
    drivePIDController.setI(iS);
    drivePIDController.setD(dS);
    drivePIDController.setIZone(izS);
    drivePIDController.setFF(ffS);
    drivePIDController.setOutputRange(minS, maxS); 
     
    drivePIDController.setP(pD);
    drivePIDController.setI(iD);
    drivePIDController.setD(dD);
    drivePIDController.setIZone(izD);
    drivePIDController.setFF(ffD);
    drivePIDController.setOutputRange(minD, maxD); 
  }
}