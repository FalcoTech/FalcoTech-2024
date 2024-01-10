// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;


import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule extends SubsystemBase {
  private final String moduleName;

  private final CANSparkMax driveMotor;
  private final CANSparkMax turnMotor;
  private final boolean driveMotorReversed;
  private final boolean turnMotorReversed;

  private final RelativeEncoder driveEncoder;

  private final PIDController turnPID;

  private final CANcoder absoluteEncoder;
  private final boolean absoluteEncoderReversed;
  private final double absoluteEncoderOffsetRad;

  /** Creates a new SwerveModule. */
    public SwerveModule(
        String moduleName,
        int driveMotorID, int turnMotorID, boolean isDriveMotorReversed, boolean isTurnMotorReversed,
        int absoluteEncoderID, double absoluteEncoderOffset, boolean isAbsoluteEncoderReversed
    ){
    
    this.moduleName = moduleName;
        
    driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
    turnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);
    
    this.driveMotorReversed = isDriveMotorReversed;
    this.turnMotorReversed = isTurnMotorReversed;
    driveMotor.setInverted(driveMotorReversed);
    turnMotor.setInverted(turnMotorReversed);
    
    driveEncoder = driveMotor.getEncoder();

    driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
    driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);

    this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
    this.absoluteEncoderReversed = isAbsoluteEncoderReversed;
    absoluteEncoder = new CANcoder(absoluteEncoderID);

    turnPID = new PIDController(ModuleConstants.kPTurning, ModuleConstants.kITurning, ModuleConstants.kDTurning);
    turnPID.enableContinuousInput(-Math.PI, Math.PI);

    resetEncoders();
  }

  public double getDrivePosition(){
    return driveEncoder.getPosition();
  }
  public double getDriveVelocity(){
    return driveEncoder.getVelocity();
  }

  public double getAbsoluteEncoderPosition(){
    double angle = absoluteEncoder.getPosition().getValueAsDouble();
    // double angle = Math.IEEEremainder(absoluteEncoder.getPosition(), 2*Math.PI);
    angle -= absoluteEncoderOffsetRad;
    return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
  }
  public double getAbsoluteEncoderVelocity(){
    double velocity = absoluteEncoder.getVelocity().getValueAsDouble();
    return velocity * (absoluteEncoderReversed ? -1.0 : 1.0);
  }

  public void resetEncoders(){
    driveEncoder.setPosition(0);
  }


  public SwerveModuleState getSwerveModuleState(){
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getAbsoluteEncoderPosition()));
  }
  public SwerveModulePosition getSwerveModulePosition(){
    return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getAbsoluteEncoderPosition()));
  }
  public void setDesiredState(SwerveModuleState state){
    if (Math.abs(state.speedMetersPerSecond) < 0.01){
      stopMotors();
      return;
    }

    state = SwerveModuleState.optimize(state, getSwerveModuleState().angle);
    driveMotor.set(state.speedMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond);
    turnMotor.set(turnPID.calculate(getAbsoluteEncoderPosition(), state.angle.getRadians()));

    SmartDashboard.putString(moduleName + " state", state.toString());    
  }
  
  public void setDesiredState(SwerveModuleState state, boolean ignoreSpeedCheck){
    if (ignoreSpeedCheck){
      state = SwerveModuleState.optimize(state, getSwerveModuleState().angle);

      driveMotor.set(state.speedMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond);
      turnMotor.set(turnPID.calculate(getAbsoluteEncoderPosition(), state.angle.getRadians()));

      SmartDashboard.putString(moduleName + " state", state.toString()); 
    } else {
      setDesiredState(state);
    }
  }


  public void brakeMotors(){
    driveMotor.setIdleMode(IdleMode.kBrake);
    turnMotor.setIdleMode(IdleMode.kBrake);
  }
  public void coastMotors(){
    driveMotor.setIdleMode(IdleMode.kCoast);
    turnMotor.setIdleMode(IdleMode.kCoast);
  }
  public void stopMotors(){
    driveMotor.set(0);
    turnMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber(moduleName + "AbsEnc Rad", getAbsoluteEncoderPosition());

    SmartDashboard.putNumber(moduleName + " ABS ENC GETPOSITION", absoluteEncoder.getPosition().getValueAsDouble());


  }
}
