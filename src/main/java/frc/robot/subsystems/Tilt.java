// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class Tilt extends SubsystemBase {
  // private final CANSparkMax leftLiftMotor;
  // private final CANSparkMax rightLiftMotor;

  // private final Encoder tiltEncoder;

  // private final PIDController tiltPID;

  /** Creates a new Tilt. */
  public Tilt() {
    // leftLiftMotor = new CANSparkMax(TiltConstants.kLeftTiltMotorID, MotorType.kBrushless);
    // rightLiftMotor = new CANSparkMax(TiltConstants.kRightTiltMotorID, MotorType.kBrushless);
    // rightLiftMotor.follow(leftLiftMotor);

    // tiltEncoder = new Encoder(TiltConstants.kTiltEncoderChannelA, TiltConstants.kTiltEncoderChannelB);

    // tiltPID = new PIDController(TiltConstants.kLiftPID_P, TiltConstants.kLiftPID_I, TiltConstants.kLiftPID_D);
  }

  // public void setTiltSpeed(double speed){
  //   leftLiftMotor.set(speed);
  // }
  // public double getTiltAngle(){
  //   return tiltEncoder.getDistance();
  // }
  // public void setTiltToPoint(double setpointdegrees){
  //   setTiltSpeed(tiltPID.calculate(getTiltAngle(), setpointdegrees));
  // }
  // public void stopTilt(){
  //   leftLiftMotor.set(0);
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
