// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ThePinkAlliance.core.drivetrain.swerve.SwerveModuleConfig;
import com.ThePinkAlliance.core.util.Gains;
import com.ThePinkAlliance.swervelib.Mk4SwerveModuleHelper;
import com.ThePinkAlliance.swervelib.SdsModuleConfigurations;
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

  public static double GLOBAL_SWERVE_POD_RAMP_RATE = 0.13;

  public static SwerveModuleConfig frontLeftConfig = new SwerveModuleConfig(
    0,
    1,
    3,
    GLOBAL_SWERVE_POD_RAMP_RATE
  );
  public static SwerveModuleConfig frontRightConfig = new SwerveModuleConfig(
    4,
    5,
    6,
    GLOBAL_SWERVE_POD_RAMP_RATE
  );
  public static SwerveModuleConfig backLeftConfig = new SwerveModuleConfig(
    7,
    8,
    9,
    GLOBAL_SWERVE_POD_RAMP_RATE
  );
  public static SwerveModuleConfig backRightConfig = new SwerveModuleConfig(
    10,
    11,
    12,
    GLOBAL_SWERVE_POD_RAMP_RATE
  );

  public static Mk4SwerveModuleHelper.GearRatio gearRatio =
    Mk4SwerveModuleHelper.GearRatio.L1;

  public static final double MAX_VOLTAGE = 12.0;

  public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(
    23.4
  );
  public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(
    23.1
  );

  public static final double MAX_VELOCITY_METERS_PER_SECOND =
    // 6380 is the theoretical max rpm (e.g. NO LOAD RPM)
    5000.0 /
    60.0 *
    SdsModuleConfigurations.MK4_L1.getDriveReduction() *
    SdsModuleConfigurations.MK4_L1.getWheelDiameter() *
    Math.PI; // 5.107;

  public static final double MAX_ACCELERATION_METERS_PER_SECOND = 4.346;
  public static double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND =
    MAX_VELOCITY_METERS_PER_SECOND /
    Math.hypot(
      DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
      DRIVETRAIN_WHEELBASE_METERS / 2.0
    );

  public static final Gains X_GAINS = new Gains(0, 0, 0);
  public static final Gains Y_GAINS = new Gains(0, 0, 0);
  public static final Gains THETA_GAINS = new Gains(0, 0, 0);
}
