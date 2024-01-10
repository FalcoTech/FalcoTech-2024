// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveBaseConstants;
import frc.robot.Constants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase {
  private final SwerveModule frontLeftModule = new SwerveModule(
    "Front Left",
    DriveBaseConstants.kFrontLeftDriveMotorID,
    DriveBaseConstants.kFrontLeftTurnMotorID,
    DriveBaseConstants.kFrontLeftDriveMotorReversed,
    DriveBaseConstants.kFrontLeftTurnMotorReversed,
    DriveBaseConstants.kFrontLeftAbsoluteEncoderID,
    DriveBaseConstants.kFrontLeftAbsoluteEncoderOffsetRadians,
    DriveBaseConstants.kFrontLeftAbsoluteEncoderReversed);

  private final SwerveModule frontRightModule = new SwerveModule(
    "Front Right",
    DriveBaseConstants.kFrontRightDriveMotorID,
    DriveBaseConstants.kFrontRightTurnMotorID,
    DriveBaseConstants.kFrontRightDriveMotorReversed,
    DriveBaseConstants.kFrontRightTurnMotorReversed,
    DriveBaseConstants.kFrontRightAbsoluteEncoderID,
    DriveBaseConstants.kFrontRightAbsoluteEncoderOffsetRadians,
    DriveBaseConstants.kFrontRightAbsoluteEncoderReversed);

  private final SwerveModule backLeftModule = new SwerveModule(
    "Back Left",
    DriveBaseConstants.kBackLeftDriveMotorID,
    DriveBaseConstants.kBackLeftTurnMotorID,
    DriveBaseConstants.kBackLeftDriveMotorReversed,
    DriveBaseConstants.kBackLeftTurnMotorReversed,
    DriveBaseConstants.kBackLeftAbsoluteEncoderID,
    DriveBaseConstants.kBackLeftAbsoluteEncoderOffsetRadians,
    DriveBaseConstants.kBackLeftAbsoluteEncoderReversed);
    
  private final SwerveModule backRightModule = new SwerveModule(
    "Back Right",
    DriveBaseConstants.kBackRightDriveMotorID,
    DriveBaseConstants.kBackRightTurnMotorID,
    DriveBaseConstants.kBackRightDriveMotorReversed,
    DriveBaseConstants.kBackRightTurnMotorReversed,
    DriveBaseConstants.kBackRightAbsoluteEncoderID,
    DriveBaseConstants.kBackRightAbsoluteEncoderOffsetRadians,
    DriveBaseConstants.kBackRightAbsoluteEncoderReversed);
  
    private final ADIS16448_IMU gyro = new ADIS16448_IMU();    
    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics, 
      getGyroRotation2d(), 
      getModulePositions(), 
      new Pose2d(0, 0, getGyroRotation2d()));

  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {
    new Thread(() -> { 
      try {
        Thread.sleep(1000); // Wait for gyro to calibrate,
        zeroGyro();         // then zero it
      } catch (Exception e){
        System.out.println(e);
      }
    }).start();

    //Configure autobuilder last
    AutoBuilder.configureHolonomic(
      this::getPose2d, //get pose
      this::resetPose, //reset pose
      this::getChassisSpeeds, //chassisspeeds supplier
      this::swerveDriveChassisSpeedsConsumer, //chassisspeeds consumer 
      new HolonomicPathFollowerConfig(
        new PIDConstants(3.0, 0.0, 0.0), // translation PID
        new PIDConstants(3.0, 0.0, 0.0), // rotation PID
        DriveConstants.kMaxSpeedMetersPerSecond, //max module speed m/s
        1.0, //drivebase radius
        new ReplanningConfig()
      ), 
      () -> {
        //mirror auto path for red side
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()){
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      }, 
      this); //swewrve subsystem
  }

  public double getGyroHeading(){
    return Math.IEEEremainder(gyro.getAngle(), 360);
  }  
  public Rotation2d getGyroRotation2d(){
    return Rotation2d.fromDegrees(getGyroHeading());
  }
  public void zeroGyro(){
    gyro.reset();
  }

  public Pose2d getPose2d(){
    return new Pose2d(odometry.getPoseMeters().getX(), odometry.getPoseMeters().getY(), getGyroRotation2d());
  }
  public void resetPose(Pose2d pose){
    odometry.resetPosition(
      getGyroRotation2d(),
      getModulePositions(),
      pose);
  }

  public void setModuleStates(SwerveModuleState[] desiredStates, boolean ignoreSpeedCheck){
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    frontLeftModule.setDesiredState(desiredStates[0], ignoreSpeedCheck);
    frontRightModule.setDesiredState(desiredStates[1], ignoreSpeedCheck);
    backLeftModule.setDesiredState(desiredStates[2], ignoreSpeedCheck);
    backRightModule.setDesiredState(desiredStates[3], ignoreSpeedCheck);
  }

  public void swerveDriveRobotRelative(double xSpdFB, double ySpdLR, double rotSpd){
    double xSpeedCommanded = xSpdFB * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedCommanded = ySpdLR * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotSpeedCommanded = rotSpd * DriveConstants.kMaxAngularSpeedRadiansPerSecond;

    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeedCommanded, ySpeedCommanded, rotSpeedCommanded);

    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    setModuleStates(moduleStates, false);
  }
  public void swerveDriveChassisSpeedsConsumer(ChassisSpeeds speeds){
    swerveDriveRobotRelative(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
  }

  public void brakeModules(){
    frontLeftModule.brakeMotors();
    frontRightModule.brakeMotors();
    backLeftModule.brakeMotors();
    backRightModule.brakeMotors();
  }
  public void coastModules(){
    frontLeftModule.coastMotors();
    frontRightModule.coastMotors();
    backLeftModule.coastMotors();
    backRightModule.coastMotors();
  }
  public void stopModules(){
    frontLeftModule.stopMotors();
    frontRightModule.stopMotors();
    backLeftModule.stopMotors();
    backRightModule.stopMotors();
  }

  public SwerveModuleState[] getModuleStates(){
    return new SwerveModuleState[]{
      frontLeftModule.getSwerveModuleState(),
      frontRightModule.getSwerveModuleState(),
      backLeftModule.getSwerveModuleState(),
      backRightModule.getSwerveModuleState()
    };
  }
  public SwerveModulePosition[] getModulePositions(){
    return new SwerveModulePosition[]{
      frontLeftModule.getSwerveModulePosition(),
      frontRightModule.getSwerveModulePosition(),
      backLeftModule.getSwerveModulePosition(),
      backRightModule.getSwerveModulePosition()
    };
  }

  public ChassisSpeeds getChassisSpeeds(){
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Robot Heading:", getGyroHeading());
  }
}
