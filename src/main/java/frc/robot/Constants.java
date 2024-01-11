// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static int kPilotPort = 0;
    public static int kCoPilotPort = 1;

    public static double kPilotDeadband = 0.05; //https://i.ytimg.com/vi/NdGzov5OU_Y/maxresdefault.jpg 
  }

  public static class DriveConstants {
    public static final double kMaxSpeedMetersPerSecond = Units.feetToMeters(16);
    public static final double kMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

    public static final double kTeleopDriveMaxAccelerationUnitsPerSecond = 5;
    public static final double kTeleopDriveMaxAngularAccelerationUnitsPerSecond = 5;

    public static final double kTeleopDriveSpeedScale = 1;
    public static final double kTeleopDriveTriggerSpeedScale = .05;

    //distance between centers of right and left wheels on robot (track width)
    public static final double kTrackWidth = Units.inchesToMeters(26.375);
    //https://upload.wikimedia.org/wikipedia/commons/5/52/Wheelbase_and_Track.png
    //distance between centers of front and back wheels on robot (wheel base)
    public static final double kWheelBase = Units.inchesToMeters(26.375);
    
    //TITANIUM TITANS SWERVE DRIVE KINEMATICS (And what I think will also work better)
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(kWheelBase/2, kTrackWidth/2),
      new Translation2d(kWheelBase/2, -kTrackWidth/2), 
      new Translation2d(-kWheelBase/2, kTrackWidth/2), 
      new Translation2d(-kWheelBase/2, -kTrackWidth/2)); 
    
  }

  public static class ModuleConstants {
    public static final double kWheelDiaMeterMeters = Units.inchesToMeters(4);
    public static final double kDriveMotorGearRatio = 1 / 6.12; // change this
    public static final double kTurnMotorGearRatio = 1 / (150/7); // change this
    
    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiaMeterMeters;
    public static final double kTurnEncoderRot2Radian = kTurnMotorGearRatio * 2 * Math.PI;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60.0;
    public static final double kTurnEncoderRPM2RadianPerSec = kTurnEncoderRot2Radian / 60.0;

    public static final double kPTurning = 0.5;
    public static final double kITurning = 0.0;
    public static final double kDTurning = 0.0;
  }

  public static class DriveBaseConstants {
    public static final int kFrontLeftDriveMotorID = 10;
    public static final int kFrontLeftTurnMotorID = 11;
    public static final boolean kFrontLeftDriveMotorReversed = true;
    public static final boolean kFrontLeftTurnMotorReversed = false;
    public static final int kFrontLeftAbsoluteEncoderID = 0;
    public static final double kFrontLeftAbsoluteEncoderOffsetRadians = -0.964871965348721; 
    public static final boolean kFrontLeftAbsoluteEncoderReversed = false;

    public static final int kFrontRightDriveMotorID = 12;
    public static final int kFrontRightTurnMotorID = 13;
    public static final boolean kFrontRightDriveMotorReversed = false;
    public static final boolean kFrontRightTurnMotorReversed = false;
    public static final int kFrontRightAbsoluteEncoderID = 1;
    public static final double kFrontRightAbsoluteEncoderOffsetRadians = -0.693357914686203; 
    public static final boolean kFrontRightAbsoluteEncoderReversed = true;

    public static final int kBackLeftDriveMotorID = 14;
    public static final int kBackLeftTurnMotorID = 15;
    public static final boolean kBackLeftDriveMotorReversed = true;
    public static final boolean kBackLeftTurnMotorReversed = false;
    public static final int kBackLeftAbsoluteEncoderID = 2;
    public static final double kBackLeftAbsoluteEncoderOffsetRadians = -3.066421397030354; 
    public static final boolean kBackLeftAbsoluteEncoderReversed = false;

    public static final int kBackRightDriveMotorID = 16;
    public static final int kBackRightTurnMotorID = 17;
    public static final boolean kBackRightDriveMotorReversed = false;
    public static final boolean kBackRightTurnMotorReversed = false;
    public static final int kBackRightAbsoluteEncoderID = 3;
    public static final double kBackRightAbsoluteEncoderOffsetRadians = -1.58306497335434; 
    public static final boolean kBackRightAbsoluteEncoderReversed = false;
  }
}
