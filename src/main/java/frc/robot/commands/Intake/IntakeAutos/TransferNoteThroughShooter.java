// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake.IntakeAutos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;

public class TransferNoteThroughShooter extends Command {
  public final Intake m_intakeSubsystem = RobotContainer.m_intakeSubsystem;

  public final Timer shootTimer = new Timer();
  /** Creates a new TransferNoteThroughShooter. */
  public TransferNoteThroughShooter() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shootTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intakeSubsystem.setTransferSpeed(1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shootTimer.stop();
    shootTimer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !m_intakeSubsystem.getNoteReady();
  }
}
