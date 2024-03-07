// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Tilt;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Tilt;

public class SetTiltAngleDegrees extends Command {
  private final Tilt m_tiltSubsystem;
  private double setpoint;
  /** Creates a new SetTiltToAngle. */
  public SetTiltAngleDegrees(double setpointDegrees) {
    this.m_tiltSubsystem = RobotContainer.m_tiltSubsystem;
    this.setpoint = setpointDegrees;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_tiltSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_tiltSubsystem.setTiltToSetpoint(setpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_tiltSubsystem.stopTilt();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_tiltSubsystem.getTiltAngle() >= (setpoint - .01) && m_tiltSubsystem.getTiltAngle() <= (setpoint + .01);
  }
}
