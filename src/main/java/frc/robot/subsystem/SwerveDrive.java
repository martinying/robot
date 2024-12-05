// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrive extends SubsystemBase {
  /** Creates a new SwerveDrive. */
  public SwerveDrive() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setModuleStates(SwerveModuleState[] states) {

  }

  public Rotation2d getMeasuredAngle() {
      // TODO Auto-generated method stub
      return new Rotation2d(0.0);
  }

  public void setModuleSstates(SwerveModuleState [] desiredModuleStates) {
      // TODO Auto-generated method stub

  }

}
