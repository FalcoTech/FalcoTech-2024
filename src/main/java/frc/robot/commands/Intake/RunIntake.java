// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class RunIntake extends Command {
  private final Intake m_intakeSubsystem;
  private final double intakeSpeed;
  /** Creates a new RunIntake. NEGATIVE speed will SUCK notes*/
  public RunIntake(Intake intakeSubsystem,
      double suckSpeed, double spitSpeed) {

    // Use addRequirements() here to declare subsystem dependencies.
    this.m_intakeSubsystem = intakeSubsystem;
    this.intakeSpeed = spitSpeed - suckSpeed;
    

    addRequirements(m_intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // m_intakeSubsystem.setIntakeSpeed(intakeSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // m_intakeSubsystem.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
