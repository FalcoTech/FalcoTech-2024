// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Swerve.SwerveSubsystem;



public class TeleOpDrive extends Command {
  private final SwerveSubsystem m_swerveSubsystem;
  private final Supplier<Double> xSpdFunction, ySpdFunction, rotSpdFunction;
  private final Supplier<Double> leftTriggerFunction, rightTriggerFunction;
  private final Supplier<Boolean> fieldRelativeFunction;
  private final SlewRateLimiter xLimiter, yLimiter, rotLimiter;

  /** Creates a new SwerveJoystickCommand */
  public TeleOpDrive(SwerveSubsystem swerveSubsystem, 
      Supplier<Double> xSpdFB, Supplier<Double> ySpdLR, Supplier<Double> rotSpd,
      Supplier<Double> leftTriggerSlowTurn, Supplier<Double> rightTriggerSlowTurn,
      Supplier<Boolean> fieldRelative) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_swerveSubsystem = swerveSubsystem;
    this.xSpdFunction = xSpdFB;
    this.ySpdFunction = ySpdLR;
    this.rotSpdFunction = rotSpd;
    this.leftTriggerFunction = leftTriggerSlowTurn;
    this.rightTriggerFunction = rightTriggerSlowTurn;
    this.fieldRelativeFunction = fieldRelative;

    this.xLimiter = new SlewRateLimiter(SwerveDriveConstants.kTeleopDriveMaxAccelerationUnitsPerSecond);
    this.yLimiter = new SlewRateLimiter(SwerveDriveConstants.kTeleopDriveMaxAccelerationUnitsPerSecond); //Dampen the acceleration
    this.rotLimiter = new SlewRateLimiter(SwerveDriveConstants.kTeleopDriveMaxAngularAccelerationUnitsPerSecond);

    addRequirements(m_swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Get the x, y, and rotation speeds from the joystick and apply the scale factor
    double xSpeed = xSpdFunction.get() * SwerveDriveConstants.kTeleopDriveSpeedScale;
    double ySpeed = ySpdFunction.get() * SwerveDriveConstants.kTeleopDriveSpeedScale;
    double leftTriggerRot = leftTriggerFunction.get() * SwerveDriveConstants.kTeleopDriveTriggerSpeedScale;
    double rightTriggerRot = rightTriggerFunction.get() * SwerveDriveConstants.kTeleopDriveTriggerSpeedScale;
    // Calculate the rotation speed from the triggers and the joystick (will use 1 if you try to hyperdrive lol)
    double rotSpeed = Math.min((rotSpdFunction.get() * SwerveDriveConstants.kTeleopDriveSpeedScale) + (leftTriggerRot - rightTriggerRot), 1.0); 

    // Apply a deadband to the x, y, and rotation speeds
    xSpeed = Math.abs(xSpeed) > OperatorConstants.kPilotDeadband ? xSpeed : 0;
    ySpeed = Math.abs(ySpeed) > OperatorConstants.kPilotDeadband ? ySpeed : 0; 
    rotSpeed = Math.abs(rotSpeed) > OperatorConstants.kPilotDeadband ? rotSpeed : 0;

    // Calculate speeds in meters per second
    xSpeed = xLimiter.calculate(xSpeed) * SwerveDriveConstants.kMaxSpeedMetersPerSecond;
    ySpeed = yLimiter.calculate(ySpeed) * SwerveDriveConstants.kMaxSpeedMetersPerSecond;
    rotSpeed = rotLimiter.calculate(rotSpeed) * SwerveDriveConstants.kMaxAngularSpeedRadiansPerSecond;

    // Create a speed command to send to the drivetrain
    ChassisSpeeds chassisSpeeds;
    if (fieldRelativeFunction.get()){
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        xSpeed, ySpeed, rotSpeed, m_swerveSubsystem.getGyroRotation2d());
    } else {
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rotSpeed);
    }

    // Send the command to the drivetrain
    SwerveModuleState[] moduleStates = SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    m_swerveSubsystem.setModuleStates(moduleStates, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerveSubsystem.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
