// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.util.LimelightHelpers;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  public Vision() {
    CameraServer.startAutomaticCapture();
  }

  public double getTX(){
    return LimelightHelpers.getTX(VisionConstants.kLimelightName);    
  }
  public double getTY(){
    return LimelightHelpers.getTY(VisionConstants.kLimelightName);
  }
  public double getTA(){
    return LimelightHelpers.getTA(VisionConstants.kLimelightName);
  }
  public boolean getTV(){
    return LimelightHelpers.getTV(VisionConstants.kLimelightName);
  }

  public void setLEDSon(){
    LimelightHelpers.setLEDMode_ForceOn(VisionConstants.kLimelightName);
  }
  public void setLEDSoff(){
    LimelightHelpers.setLEDMode_ForceOff(VisionConstants.kLimelightName);
  }
  
  public void CameraModeDriver(){
    LimelightHelpers.setCameraMode_Driver(VisionConstants.kLimelightName);
  }
  public void CameraModeVision(){
    LimelightHelpers.setCameraMode_Processor(VisionConstants.kLimelightName);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
