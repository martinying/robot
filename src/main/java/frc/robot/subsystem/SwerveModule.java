// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.constants.DriveConstants;

/** Add your docs here. */
public class SwerveModule {

    private final TalonFX driveMotorController;
    private final CANSparkMax turnMotorController;
    private final DutyCycleEncoder absoluteEncoder;
    private final double absoluteEncoderOffset;
    private final PIDController turningPIDController;
    private StatusSignal<Double> talonFXDrivePositionSignal;

    public SwerveModule(int driveId, int turnId, int absoluteEncoderPort, double absoluteEncoderOffset, boolean driveReversed, boolean turningReversed) {
        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        driveConfig.MotorOutput.Inverted = driveReversed ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveConfig.Feedback.SensorToMechanismRatio = DriveConstants.kDriveSensorToMechanismRatio;
        
        driveMotorController = new TalonFX(driveId);
        driveMotorController.getConfigurator().apply(driveConfig);
        driveMotorController.setPosition(0.0); //resetDriveMotorEncoder
        talonFXDrivePositionSignal = driveMotorController.getPosition();

        turnMotorController = new CANSparkMax(turnId, MotorType.kBrushless);
        turnMotorController.setIdleMode(IdleMode.kBrake);
        turnMotorController.setInverted(turningReversed);

        this.absoluteEncoderOffset = absoluteEncoderOffset;
        absoluteEncoder = new DutyCycleEncoder(new DigitalInput(absoluteEncoderPort));

        turningPIDController = new PIDController(DriveConstants.kPTurning, DriveConstants.kITurning,
        DriveConstants.kDTurning);
        turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }
    
    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < DriveConstants.kTranslationalDeadbandMetersPerSecond) {
            driveMotorController.stopMotor();
            turnMotorController.stopMotor();
            return;
          }
        SwerveModuleState optimizedState = SwerveModuleState.optimize(state, getAbsoluteEncoderCurrentAngle());

        driveMotorController.set(optimizedState.speedMetersPerSecond/DriveConstants.kMaxTranslationalMetersPerSecond);
        turnMotorController.set(turningPIDController.calculate(getAbsoluteEncoderCurrentAngle().getRadians(), optimizedState.angle.getRadians()));
    }

    //we use drive position directly, AdvantageKit multiplies with wheel radius??
    public SwerveModulePosition getPosition() {return new SwerveModulePosition(talonFXDrivePositionSignal.refresh().getValueAsDouble(),getAbsoluteEncoderCurrentAngle());}

    private Rotation2d getAbsoluteEncoderCurrentAngle() {
        return new Rotation2d(absoluteEncoder.getDistance() - absoluteEncoderOffset);
    }
}
