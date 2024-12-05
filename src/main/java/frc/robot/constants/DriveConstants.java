// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public final class DriveConstants {
    public static final double kMaxTranslationalMetersPerSecond = Units.feetToMeters(18.9); // used to be 18.9
    public static double kMaxAngularAccelBotRotsPerSecondSquared = 3;
    public static double kMaxTurningRadiansPerSecond = 1.5 * Math.PI;

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(0.3556 - 0.065, 0.3556 - 0.068),
            new Translation2d(0.3556 - 0.066, -0.3556 + 0.066),
            new Translation2d(0.3556 - 0.645, -0.3556 + 0.644),
            new Translation2d(0.3556 - 0.644, -0.3556 + 0.063));

}
