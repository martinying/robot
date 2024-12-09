// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DriveConstants;

public class SwerveDrive extends SubsystemBase {
  private final SwerveModule frontLeft = new SwerveModule();
  private final SwerveModule frontRight = new SwerveModule();
  private final SwerveModule backLeft = new SwerveModule();
  private final SwerveModule backRight = new SwerveModule();

  private final AHRS imu = new AHRS();
  private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics,this.getMeasuredAngle(),new SwerveModulePosition[] {
    frontLeft.getPosition(),
    frontRight.getPosition(),
    backLeft.getPosition(),
    backRight.getPosition()
  });

  /** Creates a new SwerveDrive. */
  public SwerveDrive() {
    new Thread(() -> {
      try {
        Thread.sleep(1000);
        resetHeading();
      } catch (Exception e) {
        // System.out.println("ERROR in sleep thread: " + e);
      }
     }).start();
  }

  @Override
  public void periodic() {
    odometer.update(getMeasuredAngle(), 
      new SwerveModulePosition[] {frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()});
  }

  public void setModuleStates(SwerveModuleState[] desiredModuleStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredModuleStates, DriveConstants.kMaxTranslationalMetersPerSecond);
    frontLeft.setDesiredState(desiredModuleStates[0]);
    frontRight.setDesiredState(desiredModuleStates[1]);
    backLeft.setDesiredState(desiredModuleStates[2]);
    backRight.setDesiredState(desiredModuleStates[3]);
  }

  public Rotation2d getMeasuredAngle() {
      return imu.getRotation2d();
  }

  public void resetHeading() {
    imu.reset();
  }
}
