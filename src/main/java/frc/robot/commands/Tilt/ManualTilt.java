// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Tilt;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Tilt;


public class ManualTilt extends Command {
  private final Tilt m_tiltSubsystem = RobotContainer.m_tiltSubsystem;
  private final Supplier<Double> tiltSpeed;

  /** Creates a new ManualTilt. */
  public ManualTilt(Supplier<Double> speed) {
    this.tiltSpeed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_tiltSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_tiltSubsystem.HANGSPEED){
      if (tiltSpeed.get() < 0){
        m_tiltSubsystem.moveTilt(tiltSpeed.get() * .65);
      } else if (tiltSpeed.get() > 0){
        m_tiltSubsystem.moveTilt(tiltSpeed.get() * .15);
      }
    } else {
      if (tiltSpeed.get() < 0){
        m_tiltSubsystem.moveTilt(tiltSpeed.get() * .3);
      } else if (tiltSpeed.get() > 0){
        m_tiltSubsystem.moveTilt(tiltSpeed.get() * .4);
      }
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_tiltSubsystem.stopTilt();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
