// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  // private final VictorSP FrontIntakeMotor = new VictorSP(IntakeConstants.kFrontIntakeMotorID);
  // private final VictorSP BackIntakeMotor = new VictorSP(IntakeConstants.kBackIntakeMotorID);

  // private final VictorSP LiftMotor = new VictorSP(IntakeConstants.kLiftMotorID);
  
  /** Creates a new Intake. */
  public Intake() {
    //Motor following works differently for VictorSPs, the argument of addFollower 
    //is the motor that will follow the motor that addFollower is called on
    //(in this case, the back motor will follow the front motor) 
    // FrontIntakeMotor.addFollower(BackIntakeMotor); 
    // BackIntakeMotor.setInverted(true);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
