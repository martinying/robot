// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.OIConstants;
import frc.robot.subsystem.SwerveDrive;

public class SwerveJoystick extends Command {
  /** Creates a new SwerveJoystick. */

  private final Joystick joyStick;
  private final SlewRateLimiter turningSlewRateLimiter;
  private final SlewRateLimiter xSlewRateLimiter;
  private final SlewRateLimiter ySlewRateLimiter;
  private final SwerveDrive swerveDrive;

  public SwerveJoystick(SwerveDrive swerveDrive, Joystick joyStick) {
    this.turningSlewRateLimiter = new SlewRateLimiter(DriveConstants.kMaxAngularAccelBotRotsPerSecondSquared);
    this.xSlewRateLimiter = new SlewRateLimiter(DriveConstants.kMaxTranslationalMetersPerSecond);
    this.ySlewRateLimiter = new SlewRateLimiter(DriveConstants.kMaxTranslationalMetersPerSecond);
    this.joyStick = joyStick;
    this.swerveDrive = swerveDrive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {System.out.println("SwerveJoystick Initialized");}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = joyStick.getRawAxis(0);
    double ySpeed = joyStick.getRawAxis(1);
    double turningSpeed = joyStick.getRawAxis(2);

    SmartDashboard.putNumber("Joystick/xSpeedRaw", xSpeed);
    SmartDashboard.putNumber("Joystick/ySpeedRaw", ySpeed);
    SmartDashboard.putNumber("Joystick/turningSpeedRaw", turningSpeed);

    //apply deadband
    xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
    turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

    //use SlewRateLimiter with DriveConstants
    xSpeed = xSlewRateLimiter.calculate(xSpeed) * DriveConstants.kMaxTranslationalMetersPerSecond;
    ySpeed = ySlewRateLimiter.calculate(ySpeed) * DriveConstants.kMaxTranslationalMetersPerSecond;
    turningSpeed = turningSlewRateLimiter.calculate(turningSpeed) * DriveConstants.kMaxTurningRadiansPerSecond;
    SmartDashboard.putNumber("Joystick/xSpeedCommanded", xSpeed);
    SmartDashboard.putNumber("Joystick/ySpeedCommanded", ySpeed);
    SmartDashboard.putNumber("Joystick/turningSpeedCommanded", turningSpeed);

    SwerveModuleState [] desiredSwerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed,ySpeed,turningSpeed, swerveDrive.getMeasuredAngle()));
    Logger.recordOutput(getName(), desiredSwerveModuleStates);
    swerveDrive.setModuleSstates(desiredSwerveModuleStates);
    
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
