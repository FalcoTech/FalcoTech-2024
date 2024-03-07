// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter.ShooterAutos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RunShooterWithSpeed extends InstantCommand {
  public final Shooter m_shooterSubsystem = RobotContainer.m_shooterSubsystem;

  public double shooterSpeed;

  public RunShooterWithSpeed(double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooterSubsystem);

    this.shooterSpeed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooterSubsystem.setShooterSpeed(shooterSpeed);
  }
}
