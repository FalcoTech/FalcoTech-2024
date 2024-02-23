// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

public class RunShooter extends Command {
  private final Shooter m_ShooterSubsystem = RobotContainer.m_shooterSubsystem;
  private final Vision m_VisionSubsystem = RobotContainer.m_visionSubsystem;
  private double shooterSpeed;
  /** Creates a new RunShooter. POSITIVE speed will SHOOT note*/
  public RunShooter(double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooterSpeed = speed;

    addRequirements(m_ShooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ShooterSubsystem.setShooterSpeed(shooterSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ShooterSubsystem.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
