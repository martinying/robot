// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class DriveConstants {
    //physical characteristics
    public static final double kMaxTranslationalMetersPerSecond = Units.feetToMeters(18.9);
    public static final double kMaxAngularAccelBotRotsPerSecondSquared = 3;
    public static final double kMaxTurningRadiansPerSecond = 1.5 * Math.PI;

    //front left swerve module input
    public static final int kFrontLeftAbsoluteEncoderPort = 9;
    public static final double kFrontLeftAbsoluteEncoderOffset = -0.644990463821544; //why is there no + (Math.PI / 2.0);
    public static final int kFrontLeftDriveId = 51;
    public static final int kFrontLeftTurnId = 1;
    public static final boolean kFrontLeftDriveReversed = true;
    public static final boolean kFrontLeftTurningReversed = true;
    //front right swerve module input
    public static final int kFrontRightAbsoluteEncoderPort = 7;
    public static final double kFrontRightAbsoluteEncoderOffset = 1.412447275814819 + (Math.PI / 2.0);
    public static final int kFrontRightDriveId = 52;
    public static final int kFrontRightTurnId = 3;
    public static final boolean kFrontRightDriveReversed = true;
    public static final boolean kFrontRightTurningReversed = true;
    //back left swerve module input
    public static final int kBackLeftAbsoluteEncoderPort = 8;
    public static final double kBackLeftAbsoluteEncoderOffset = 1.286610661599266 + (Math.PI / 2.0);
    public static final int kBackLeftDriveId = 53;
    public static final int kBackLeftTurnId = 7;
    public static final boolean kBackLeftDriveReversed = false;
    public static final boolean kBackLeftTurningReversed = true;
    //back right swerve module input
    public static final int kBackRightAbsoluteEncoderPort = 6;
    public static final double kBackRightAbsoluteEncoderOffset = -0.406511594330128 - (Math.PI / 2.0);
    public static final int kBackRightDriveId = 54;
    public static final int kBackRightTurnId = 5;
    public static final boolean kBackRightDriveReversed = true;
    public static final boolean kBackRightTurningReversed = false; //why is it not reversed?

    //drive motor configuration
    public static final double kDriveGearRatio = 5.9028;
    public static final double kDriveSensorToMechanismRatio = kDriveGearRatio / Units.inchesToMeters(4 * 0.95 * Math.PI); //wheel diameter is 4; 0.95 here is to correct some error. There was a -5% measured error in real life. No Magic Numbers!

    //swerve module control system configurations
    public static final double kPTurning = 0.5;
    public static final double kITurning = 0.0;
    public static final double kDTurning = 0.0;

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(0.3556 - 0.065, 0.3556 - 0.068),
            new Translation2d(0.3556 - 0.066, -0.3556 + 0.066),
            new Translation2d(0.3556 - 0.645, -0.3556 + 0.644),
            new Translation2d(0.3556 - 0.644, -0.3556 + 0.063));
    public static final double kTranslationalDeadbandMetersPerSecond = 0;
}    
