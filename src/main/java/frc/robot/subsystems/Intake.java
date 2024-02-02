// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  // private final CANSparkMax frontIntakeMotor;
  // private final CANSparkMax backIntakeMotor;
  /** Creates a new Intake. */
  public Intake() {
    // frontIntakeMotor = new CANSparkMax(IntakeConstants.kFrontIntakeMotorID, CANSparkMax.MotorType.kBrushless);
    // backIntakeMotor = new CANSparkMax(IntakeConstants.kBackIntakeMotorID, CANSparkMax.MotorType.kBrushless);

    // frontIntakeMotor.setInverted(false);
    // backIntakeMotor.follow(frontIntakeMotor, false);
  }

  // public void setIntakeSpeed(double speed){
  //   frontIntakeMotor.set(speed);
  // }
  // public void stopIntake(){
  //   frontIntakeMotor.set(0);
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
