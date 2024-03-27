// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter.ShooterAutos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;

public class RunShooterForTime extends Command {
  private final Shooter m_shooterSubsystem = RobotContainer.m_shooterSubsystem;
  private Timer shooterTimer = new Timer();

  private double shootSpeed;
  private double secToRun;
  /** Creates a new RunShooterForTime. */
  public RunShooterForTime(double speed, double seconds) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooterSubsystem);
    this.shootSpeed = speed;
    this.secToRun = seconds;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterTimer.stop();
    shooterTimer.reset();
    shooterTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooterSubsystem.setShooterSpeed(shootSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterTimer.stop();
    shooterTimer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shooterTimer.get() > secToRun;
  }
}
