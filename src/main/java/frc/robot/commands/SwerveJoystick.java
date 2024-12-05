// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.OIConstants;

public class SwerveJoystick extends Command {
  /** Creates a new SwerveJoystick. */

  private final Supplier<Double> xSpeed, ySpeed, turningSpeed;
  private final SlewRateLimiter turningSlewRateLimiter;

  public SwerveJoystick(Supplier<Double> xSpeed, Supplier<Double> ySpeed, Supplier<Double> turningSpeed) {
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.turningSpeed = turningSpeed;
        
    this.turningSlewRateLimiter = new SlewRateLimiter(DriveConstants.kMaxAngularAccelBotRotsPerSecondSquared);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = this.xSpeed.get();
    double ySpeed = this.ySpeed.get();
    double turningSpeed = this.turningSpeed.get();

    //apply deadband
    xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
    turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

    xSpeed *= DriveConstants.kMaxTranslationalMetersPerSecond;
    ySpeed *= DriveConstants.kMaxTranslationalMetersPerSecond;
    turningSpeed = turningSlewRateLimiter.calculate(turningSpeed) * DriveConstants.kMaxTurningRadiansPerSecond;
    SmartDashboard.putNumber("Swerve/turningSpeedCommanded", turningSpeed);

    DriveConstants.kDriveKinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed,ySpeed,turningSpeed, null));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
