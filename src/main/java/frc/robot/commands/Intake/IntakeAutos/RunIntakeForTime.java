// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake.IntakeAutos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;

public class RunIntakeForTime extends Command {
  private Timer intakeTimer = new Timer();
  private final Intake m_intakeSubsystem = RobotContainer.m_intakeSubsystem;
  private double intakeSpeed;
  private double secToRun;
  /** Creates a new RunIntakeForTime. */
  public RunIntakeForTime(double speed, double seconds) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intakeSubsystem);
    this.intakeSpeed = speed;
    this.secToRun = seconds;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeTimer.stop();
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
    intakeTimer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intakeTimer.get() > secToRun;
  }
}
