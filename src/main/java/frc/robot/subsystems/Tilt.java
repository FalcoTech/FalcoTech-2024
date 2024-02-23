// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class Tilt extends SubsystemBase {
  private final CANSparkMax leftTiltMotor = new CANSparkMax(TiltConstants.kLeftTiltMotorID, MotorType.kBrushless);
  private final CANSparkMax rightTiltMotor = new CANSparkMax(TiltConstants.kRightTiltMotorID, MotorType.kBrushless);

  private final DutyCycleEncoder tiltEncoder = new DutyCycleEncoder(TiltConstants.kTiltEncoderDIOPort);

  private final PIDController m_tiltPID = new PIDController(TiltConstants.kLiftPID_P, TiltConstants.kLiftPID_I, TiltConstants.kLiftPID_D);


  /** Creates a new Tilt. */
  public Tilt() {
    leftTiltMotor.setIdleMode(IdleMode.kBrake);
    rightTiltMotor.setIdleMode(IdleMode.kBrake);

    leftTiltMotor.setInverted(false);
    rightTiltMotor.follow(leftTiltMotor, true);
  }
  
  public void moveTilt(double speed){
    leftTiltMotor.set(speed);
  }
  public void stopTilt(){
    leftTiltMotor.set(0);
  }

  public double getTiltAngle(){
    return tiltEncoder.get();
  }
  public void setTiltToSetpoint(double setpointDegrees){
    leftTiltMotor.set(m_tiltPID.calculate(getTiltAngle(), setpointDegrees));
  }

  public void resetTiltEncoder(){
    tiltEncoder.reset();
  } 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Tilt Encoder Angle", getTiltAngle());
    SmartDashboard.putNumber("Tilt Encoder Setpoint", m_tiltPID.getSetpoint());
    SmartDashboard.putBoolean("Tilt Encoder Connected", tiltEncoder.isConnected());

    SmartDashboard.putNumber("LT Output %", leftTiltMotor.getAppliedOutput());
    SmartDashboard.putNumber("RT Output %", rightTiltMotor.getAppliedOutput());
    SmartDashboard.putNumber("LT Amp Draw ", leftTiltMotor.getOutputCurrent());
    SmartDashboard.putNumber("RT Amp Draw ", rightTiltMotor.getOutputCurrent());

    SmartDashboard.putData("Reset Tilt Encoder", new InstantCommand(() -> resetTiltEncoder()).ignoringDisable(true));
  }
}
