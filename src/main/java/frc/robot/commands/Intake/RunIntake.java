// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class RunIntake extends Command {
  private final Intake m_intakeSubsystem;
  private final double intakeSpeed;
  /** Command to run intake motors based on copilot input. NEGATIVE speed will SUCK notes*/
  public RunIntake(Intake intakeSubsystem,
      double speed) {

    // Use addRequirements() here to declare subsystem dependencies.
    this.m_intakeSubsystem = intakeSubsystem;
    this.intakeSpeed = speed;
    

    addRequirements(m_intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
