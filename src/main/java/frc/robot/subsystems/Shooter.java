// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ResourceBundle.Control;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  // private final TalonFX leftShootMotor;
  // private final TalonFX rightShootMotor;

  public Shooter() {
    // leftShootMotor = new TalonFX(ShooterConstants.kLeftShootMotorID);
    // rightShootMotor = new TalonFX(ShooterConstants.kRightShootMotorID);

    // leftShootMotor.setInverted(false);
    // rightShootMotor.setControl(new Follower(leftShootMotor.getDeviceID(), false));

  }

  // public void setShooterSpeed(double speed){
  //   leftShootMotor.set(speed);
  // }
  // public void stopShooter(){
  //   leftShootMotor.set(0);
  // }
  // public double getShooterSpeed() {
  //   double leftMotorSpeed = leftShootMotor.getVelocity().refresh().getValueAsDouble();
  //   double rightMotorSpeed = rightShootMotor.getVelocity().refresh().getValueAsDouble();
  //   return (leftMotorSpeed + rightMotorSpeed) / 2;
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Shooter Speed", getShooterSpeed());
    // SmartDashboard.putNumber("Left Shooter Speed", leftShootMotor.getVelocity().refresh().getValueAsDouble());
    // SmartDashboard.putNumber("Right Shooter Speed", rightShootMotor.getVelocity().refresh().getValueAsDouble());
  }
}
