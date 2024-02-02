// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.Swerve.TeleOpDrive;
import frc.robot.commands.Intake.RunIntake;
import frc.robot.commands.Shooter.RunShooter;
import frc.robot.commands.Swerve.LockWheels;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.subsystems.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined and initialized here...
  private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();
  private final Vision m_visionSubsystem = new Vision();
  private final Intake m_intakeSubsystem = new Intake();
  private final Shooter m_shooterSubsystem = new Shooter();

  
  private final XboxController Pilot = new XboxController(OperatorConstants.kPilotPort);
  // public final PS4Controller Pilot = new PS4Controller(OperatorConstants.kPilotPort);
  private final XboxController CoPilot = new XboxController(OperatorConstants.kCoPilotPort);

  SendableChooser<Command> m_autoChooser = new SendableChooser<>();
     
  public RobotContainer() {
    // Pathplanner Warning:
      // Named commands must be registered before the creation of any PathPlanner Autos or Paths. 
      // It is recommended to do this in RobotContainer, after subsystem initialization, but before the creation 
      // of any other commands.
    registerNamedCommands();

    configurePilotBindings();
    configureCoPilotBindings();
    configureSmartDashboard();
  }

  //TODO test joysticks to see where positive is. 
  // new Trigger(() -> Operator.getButton()).onTrue(new Command(arguments)); //Template for creating a new command binding
  private void configurePilotBindings() {
    m_swerveSubsystem.setDefaultCommand(new TeleOpDrive( 
      m_swerveSubsystem, 
      () -> -Pilot.getLeftY(), //-Pilot.getLeftY()
      () -> Pilot.getLeftX(),
      () -> -Pilot.getRightX(),
      () -> Pilot.getLeftTriggerAxis(),
      () -> Pilot.getRightTriggerAxis(),
      () -> !Pilot.getRightBumper()));

    new Trigger(() -> Pilot.getStartButton()).onTrue(new InstantCommand(() -> m_swerveSubsystem.zeroGyro())); //XBOX CONTROLLER
    // new Trigger(() -> Pilot.getOptionsButton()).onTrue(new InstantCommand(() -> m_swerveSubsystem.zeroGyro())); //PS4 CONTROLLER

    new Trigger(() -> Pilot.getYButton()).whileTrue(new LockWheels(m_swerveSubsystem)); //XBOX
    // new Trigger(() -> Pilot.getTriangleButton()).whileTrue(new SwerveLockWheels(m_swerveSubsystem)); //PS4
  }

  private void configureCoPilotBindings(){
    m_intakeSubsystem.setDefaultCommand(new RunIntake(
      m_intakeSubsystem, 
      CoPilot.getLeftTriggerAxis(),
      CoPilot.getRightTriggerAxis()));

    new Trigger(() -> CoPilot.getBButton()).onTrue(new RunShooter(m_shooterSubsystem, ShooterConstants.kShooterSpeakerSpeed)); //Shoot to Speaker
    new Trigger(() -> CoPilot.getAButton()).onTrue(new RunShooter(m_shooterSubsystem, ShooterConstants.kShooterAmpSpeed)); //Place to Amp
  }



  private void registerNamedCommands(){
    //SWERVE
    NamedCommands.registerCommand("Lock Wheels", new LockWheels(m_swerveSubsystem));

    //INTAKE

    //SHOOTER
    NamedCommands.registerCommand("Shoot Speaker", new RunShooter(m_shooterSubsystem, ShooterConstants.kShooterSpeakerSpeed));
    NamedCommands.registerCommand("Shoot Amp", new RunShooter(m_shooterSubsystem, ShooterConstants.kShooterAmpSpeed));
  }

  private void configureSmartDashboard(){
    m_autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", m_autoChooser);
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return m_autoChooser.getSelected();
  }
}
