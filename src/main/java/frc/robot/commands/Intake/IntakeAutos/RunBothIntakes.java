// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake.IntakeAutos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;

public class RunBothIntakes extends Command {
  private final Intake m_intakeSubsystem = RobotContainer.m_intakeSubsystem;
  private double intakeSpeed;
  private Timer intakeTimer = new Timer();
  /** Will run both the front intake and the transfer motor with a specified speed. POSITIVE speed will SUCK note, NEGATIVE speed will SPIT */
  public RunBothIntakes(double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakeSpeed = speed;
    addRequirements(m_intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeTimer.reset();
    intakeTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intakeSubsystem.setIntakeSpeed(intakeSpeed);
    m_intakeSubsystem.setTransferSpeed(intakeSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.stopIntake();
    m_intakeSubsystem.stopTransfer();
    intakeTimer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intakeTimer.get() > 5;
  }
}
