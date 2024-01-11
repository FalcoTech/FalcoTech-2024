// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Swerve.SwerveJoystickCommand;
import frc.robot.commands.Swerve.SwerveLockWheels;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

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
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();
  private final Vision m_vision = new Vision();

  // private final XboxController Pilot = new XboxController(0);
  public final PS4Controller Pilot = new PS4Controller(OperatorConstants.kPilotPort);

  SendableChooser<Command> m_autoChooser = new SendableChooser<>();
     
  public RobotContainer() {
    m_swerveSubsystem.setDefaultCommand(new SwerveJoystickCommand( //PS4 CONTROLLER
      m_swerveSubsystem, 
      () -> -Pilot.getLeftY(), //-Pilot.getLeftY()
      () -> Pilot.getLeftX(),
      () -> -Pilot.getRightX(),
      () -> Pilot.getL2Axis(),
      () -> Pilot.getR2Axis(),
      () -> !Pilot.getR1Button()));
    
      // Warning:
      // Named commands must be registered before the creation of any PathPlanner Autos or Paths. 
      // It is recommended to do this in RobotContainer, after subsystem initialization, but before the creation 
      // of any other commands.
    configureNamedCommands();
    configureBindings();
    configureSmartDashboard();
  }

  private void configureBindings() {
    // new Trigger(() -> Pilot.getStartButton()).onTrue(new InstantCommand(() -> m_swerveSubsystem.zeroGyro())); //XBOX CONTROLLER
    new Trigger(() -> Pilot.getOptionsButton()).onTrue(new InstantCommand(() -> m_swerveSubsystem.zeroGyro())); //PS4 CONTROLLER

    
    // new Trigger(() -> Pilot.getTriangleButton()).whileTrue(new SwerveLockWheels(m_swerveSubsystem)); 
  }

  private void configureNamedCommands(){
    NamedCommands.registerCommand("Lock Wheels", new SwerveLockWheels(m_swerveSubsystem));
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
