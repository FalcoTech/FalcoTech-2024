// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve.*;

public class SwerveLockWheels extends Command {
  private static final Rotation2d fortyFiveDegrees = Rotation2d.fromDegrees(45);

  private static final Rotation2d zeroDegrees = Rotation2d.fromDegrees(0);
  private static final Rotation2d ninetyDegrees = Rotation2d.fromDegrees(90);

  private static final SwerveModuleState[] XStanceStates = new SwerveModuleState[] {
    new SwerveModuleState(0.0, fortyFiveDegrees.unaryMinus()),
    new SwerveModuleState(0.0, fortyFiveDegrees),
    new SwerveModuleState(0.0, fortyFiveDegrees),
    new SwerveModuleState(0.0, fortyFiveDegrees.unaryMinus())
  };
  private static final SwerveModuleState[] TStanceStates = new SwerveModuleState[] {
    new SwerveModuleState(0.0, zeroDegrees),
    new SwerveModuleState(0.0, ninetyDegrees),
    new SwerveModuleState(0.0, zeroDegrees),
    new SwerveModuleState(0.0, ninetyDegrees)
  };

  private final SwerveSubsystem m_swerveSubsystem;
  /** Creates a new SwerveXStanceCommand. */

  public SwerveLockWheels(SwerveSubsystem swerveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_swerveSubsystem = swerveSubsystem;
    addRequirements(m_swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_swerveSubsystem.brakeModules();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      m_swerveSubsystem.setModuleStates(XStanceStates, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerveSubsystem.stopModules();
    m_swerveSubsystem.coastModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
