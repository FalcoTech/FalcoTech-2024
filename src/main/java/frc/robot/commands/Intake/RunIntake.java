// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class RunIntake extends Command {
  private final Intake m_intakeSubsystem = RobotContainer.m_intakeSubsystem;
  private final Shooter m_shooterSubsystem = RobotContainer.m_shooterSubsystem;
  private Supplier<Double> intakeSpeed;
  private boolean transferStage = false;
  /** Command to run intake motors based on copilot input. NEGATIVE speed will SUCK notes*/
  public RunIntake(Supplier<Double> speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakeSpeed = speed;
    
    addRequirements(m_intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (SmartDashboard.getBoolean("LASERCAN CHECKS", true) && intakeSpeed.get() > 0){
    //   if (!m_intakeSubsystem.getTransferReady() && !m_intakeSubsystem.getNoteReady() && !transferStage){
    //     m_intakeSubsystem.setIntakeSpeed(intakeSpeed.get());
    //   } else if (m_intakeSubsystem.getTransferReady() && !m_intakeSubsystem.getNoteReady()){
    //     transferStage = true;
    //     m_intakeSubsystem.setIntakeSpeed(intakeSpeed.get() * .2);
    //     m_intakeSubsystem.setTransferSpeed(intakeSpeed.get());
    //   } else if (!m_intakeSubsystem.getTransferReady() && !m_intakeSubsystem.getNoteReady() && transferStage){
    //     m_intakeSubsystem.setTransferSpeed(intakeSpeed.get());
    //   } else if (!m_intakeSubsystem.getTransferReady() && m_intakeSubsystem.getNoteReady()){
    //     transferStage = false;
    //     m_intakeSubsystem.stopIntake();
    //     if (m_shooterSubsystem.getShooterSpeed() > 1000){
    //       m_intakeSubsystem.setTransferSpeed(intakeSpeed.get());
    //     }
    //   }
    // } else {
    //   m_intakeSubsystem.setIntakeSpeed(intakeSpeed.get());
    //   m_intakeSubsystem.setTransferSpeed(intakeSpeed.get());
    // }

    m_intakeSubsystem.setIntakeSpeed(intakeSpeed.get());
    m_intakeSubsystem.setTransferSpeed(intakeSpeed.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.stopIntake();
    m_intakeSubsystem.stopTransfer();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
