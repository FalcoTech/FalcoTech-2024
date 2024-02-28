// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.Swerve.TeleOpDrive;
import frc.robot.commands.Tilt.ManualTilt;
import frc.robot.commands.Tilt.SetTiltAngleDegrees;
import frc.robot.commands.Intake.RunIntake;
import frc.robot.commands.Shooter.RunShooter;
import frc.robot.commands.Swerve.LockWheels;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.subsystems.*;

import com.fasterxml.jackson.databind.util.PrimitiveArrayBuilder;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined and initialized here
  public static final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();
  public static final Vision m_visionSubsystem = new Vision();
  public static final Intake m_intakeSubsystem = new Intake();
  public static final Tilt m_tiltSubsystem = new Tilt();
  public static final Shooter m_shooterSubsystem = new Shooter();

  
  private final XboxController Pilot = new XboxController(OperatorConstants.kPilotPort);
  // public final PS4Controller Pilot = new PS4Controller(OperatorConstants.kPilotPort);
  private final XboxController CoPilot = new XboxController(OperatorConstants.kCoPilotPort);

  // The chooser for the autonomous routines
  SendableChooser<Command> m_autoChooser = new SendableChooser<>();
     
  public RobotContainer() {
    // Pathplanner Warning:
      // Named commands must be registered before the creation of any PathPlanner Autos or Paths. 
      // It is recommended to do this in RobotContainer, after subsystem initialization, but before the creation 
      // of any other commands
    registerNamedCommands();

    configurePilotBindings();
    configureCoPilotBindings();
    configureSmartDashboard();
  }

  // new Trigger(() -> OPERATOR.GETSOMEBUTTON()).onTrue(new COMMAND(ARGUMENTS)); //Template for creating a new command binding
  private void configurePilotBindings() {
    m_swerveSubsystem.setDefaultCommand(new TeleOpDrive( 
      //TODO: Change x and y back and test because I'm stupid - Gavin
      () -> -Pilot.getLeftX(),
      () -> -Pilot.getLeftY(),
      () -> -Pilot.getRightX(),
      () -> 0.0, //Pilot.getRightTriggerAxis()
      () -> 0.0, //Pilot.getLeftTriggerAxis()
      () -> !Pilot.getRightBumper(), //Field Relative
      () -> Pilot.getXButton())); //Vision Align

    new Trigger(() -> Pilot.getStartButton()).onTrue(new InstantCommand(() -> m_swerveSubsystem.zeroGyro())); //XBOX CONTROLLER
    new Trigger(() -> Pilot.getAButton()).onTrue(new InstantCommand(() -> m_swerveSubsystem.brakeModules())); //XBOX CONTROLLER
    new Trigger(() -> Pilot.getBButton()).onTrue(new InstantCommand(() -> m_swerveSubsystem.coastModules())); //XBOX CONTROLLER
    // new Trigger(() -> Pilot.getOptionsButton()).onTrue(new InstantCommand(() -> m_swerveSubsystem.zeroGyro())); //PS4 CONTROLLER

    new Trigger(() -> Pilot.getYButton()).whileTrue(new LockWheels()); //XBOX
    // new Trigger(() -> Pilot.getTriangleButton()).whileTrue(new SwerveLockWheels()); //PS4
  }

  private void configureCoPilotBindings(){
    m_intakeSubsystem.setDefaultCommand(new RunIntake((() -> (CoPilot.getRightTriggerAxis() - CoPilot.getLeftTriggerAxis()))));
    new Trigger(() -> CoPilot.getXButton()).whileTrue(new RunIntake(() -> 1.0));


    m_tiltSubsystem.setDefaultCommand(new ManualTilt(() -> CoPilot.getRightY()));
    new Trigger(() -> CoPilot.getYButton()).whileTrue(new SetTiltAngleDegrees(.3)); //Shoot to Speaker



    new Trigger(() -> CoPilot.getBButton()).whileTrue(new RunShooter(ShooterConstants.kShooterSpeakerSpeed)); //Shoot to Speaker
    new Trigger(() -> CoPilot.getAButton()).whileTrue(new RunShooter(ShooterConstants.kShooterAmpSpeed)); //Place to Amp

    //Multiple commands can be bound to the same button using command groups
  }


  private void registerNamedCommands(){
    //SWERVE
    NamedCommands.registerCommand("Lock Wheels", new LockWheels());

    //INTAKE

    //SHOOTER
    NamedCommands.registerCommand("Shoot Speaker", new RunShooter(ShooterConstants.kShooterSpeakerSpeed));
    NamedCommands.registerCommand("Shoot Amp", new RunShooter(ShooterConstants.kShooterAmpSpeed));

    //TILT
  }

  private void configureSmartDashboard(){
    m_autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", m_autoChooser); //send auto chooser to smartdashboard
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected(); //return the selected auto command
  }
}
