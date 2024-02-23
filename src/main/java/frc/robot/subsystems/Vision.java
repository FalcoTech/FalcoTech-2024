// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.VisionConstants;
import frc.robot.util.LimelightHelpers;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  public Vision() {
    //Start USB Camera on RoboRIO
    CameraServer.startAutomaticCapture();
  }

  public double getTX(){
    return LimelightHelpers.getTX(""); //TODO: check if blank works   
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

  public double getDistanceToTarget(){
    double verticalOffset = getTY();
    double angleToGoalRadians = Units.degreesToRadians(verticalOffset + VisionConstants.kLimelightMountAngleDegrees);

    return (VisionConstants.kApriltagHeight - VisionConstants.kLimelightMountHeight) / Math.tan(angleToGoalRadians);
  }

  public double[] getTargetPose(){
    return LimelightHelpers.getTargetPose_CameraSpace("");
  }

  
  public double getTagID(){
    return LimelightHelpers.getFiducialID("");
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
    SmartDashboard.putNumber("Limelight tX", getTX());
    SmartDashboard.putNumber("Limelight tY", getTY());
    SmartDashboard.putBoolean("Limelight Valid Target", getTV());

    SmartDashboard.putNumber("Limelight Distance to Target", getDistanceToTarget());
    
  }
}
