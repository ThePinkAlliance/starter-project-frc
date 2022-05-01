// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ThePinkAlliance.swervelib.Mk4SwerveModuleHelper;
import com.ThePinkAlliance.swervelib.SwerveModule;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.Supplier;

public class Base extends SubsystemBase {

  AHRS gyro;

  SwerveModule frontLeftModule;
  SwerveModule frontRightModule;
  SwerveModule backLeftModule;
  SwerveModule backRightModule;

  ChassisSpeeds chassisSpeeds;

  SwerveDriveKinematics kinematics;

  SwerveDriveOdometry odometry;

  SwerveModuleState[] states;

  ShuffleboardTab tab = Shuffleboard.getTab("debug");

  /** Creates a new Base. */
  public Base() {
    this.chassisSpeeds = new ChassisSpeeds();

    this.gyro = new AHRS();

    this.kinematics =
      new SwerveDriveKinematics(
        // Front Left Pod
        new Translation2d(
          Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
          Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0
        ),
        // Front Right
        new Translation2d(
          Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
          -Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0
        ),
        // Back Left
        new Translation2d(
          -Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
          Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0
        ),
        // Back Right
        new Translation2d(
          -Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
          -Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0
        )
      );

    this.odometry =
      new SwerveDriveOdometry(
        kinematics,
        Rotation2d.fromDegrees(gyro.getFusedHeading())
      );

    this.states = kinematics.toSwerveModuleStates(new ChassisSpeeds());

    this.frontLeftModule =
      Mk4SwerveModuleHelper.createFalcon500(
        tab
          .getLayout("Front Left Module", BuiltInLayouts.kList)
          .withSize(2, 4)
          .withPosition(0, 0),
        Constants.gearRatio,
        Constants.frontLeftConfig.motorPowerPort,
        Constants.frontLeftConfig.motorSteerPort,
        Constants.frontLeftConfig.canIDPort,
        Constants.frontLeftConfig.steerOffset
      );

    this.frontRightModule =
      Mk4SwerveModuleHelper.createFalcon500(
        tab
          .getLayout("Front Right Module", BuiltInLayouts.kList)
          .withSize(2, 4)
          .withPosition(0, 2),
        Constants.gearRatio,
        Constants.frontRightConfig.motorPowerPort,
        Constants.frontRightConfig.motorSteerPort,
        Constants.frontRightConfig.canIDPort,
        Constants.frontRightConfig.steerOffset
      );

    this.backRightModule =
      Mk4SwerveModuleHelper.createFalcon500(
        tab
          .getLayout("Back Right Module", BuiltInLayouts.kList)
          .withSize(2, 4)
          .withPosition(0, 4),
        Constants.gearRatio,
        Constants.backRightConfig.motorPowerPort,
        Constants.backRightConfig.motorSteerPort,
        Constants.backRightConfig.canIDPort,
        Constants.backRightConfig.steerOffset
      );

    this.backLeftModule =
      Mk4SwerveModuleHelper.createFalcon500(
        tab
          .getLayout("Back Left Module", BuiltInLayouts.kList)
          .withSize(2, 4)
          .withPosition(0, 6),
        Constants.gearRatio,
        Constants.backLeftConfig.motorPowerPort,
        Constants.backLeftConfig.motorSteerPort,
        Constants.backLeftConfig.canIDPort,
        Constants.backLeftConfig.steerOffset
      );
  }

  public void drive(ChassisSpeeds speeds) {
    states = kinematics.toSwerveModuleStates(speeds);
  }

  public void setStates(SwerveModuleState... states) {
    odometry.update(gyro.getRotation2d(), states);

    SwerveDriveKinematics.desaturateWheelSpeeds(
      states,
      Constants.MAX_VELOCITY_METERS_PER_SECOND
    );

    frontLeftModule.set(
      convertModuleSpeed(states[0].speedMetersPerSecond),
      states[0].angle.getRadians()
    );

    frontRightModule.set(
      convertModuleSpeed(states[1].speedMetersPerSecond),
      states[1].angle.getRadians()
    );

    backLeftModule.set(
      convertModuleSpeed(states[2].speedMetersPerSecond),
      states[2].angle.getRadians()
    );

    backRightModule.set(
      convertModuleSpeed(states[3].speedMetersPerSecond),
      states[3].angle.getRadians()
    );
  }

  private double convertModuleSpeed(double speedMetersPerSecond) {
    return (
      (speedMetersPerSecond / Constants.MAX_VELOCITY_METERS_PER_SECOND) *
      Constants.MAX_VOLTAGE
    );
  }

  /**
   * @return ChassisSpeeds of the robot
   */
  public ChassisSpeeds getChassisSpeeds() {
    return chassisSpeeds;
  }

  /**
   * @return Kinematics of the robot
   */
  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }

  /**
   * This resets the gyroscope's Yaw axis to zero.
   */
  public void zeroGyro() {
    gyro.reset();
  }

  /**
   * This resets the odometry to the given position and sets the rotation to the
   * current one from the gyro.
   */
  public void resetOdometry(Pose2d pose) {
    this.odometry.resetPosition(pose, this.getRotation());
  }

  /**
   * This resets the odometry to the given position and sets the rotation to the
   * current one from the gyro.
   */
  public void resetOdometry(Pose2d pose, Rotation2d rot) {
    this.odometry.resetPosition(pose, rot);
  }

  /**
   * Returns the robot's current rotation.
   *
   * @return the robot's current rotation.
   */
  public Rotation2d getRotation() {
    if (gyro.isMagnetometerCalibrated()) {
      return Rotation2d.fromDegrees(gyro.getFusedHeading());
    }

    return Rotation2d.fromDegrees(360.0 - gyro.getYaw());
  }

  public double getSensorYaw() {
    return gyro.getYaw();
  }

  /**
   * Returns if the robot inverted.
   */
  public boolean isInverted() {
    return (
      getRotation().getDegrees() <= 190 &&
      getRotation().getDegrees() > 90 ||
      getRotation().getDegrees() >= 290 &&
      getRotation().getDegrees() < 90
    );
  }

  /**
   * Returns the current odometry pose of the robot.
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Returns the current odometry pose of the robot as a supplier.
   */
  public Supplier<Pose2d> getPoseSupplier() {
    return () -> odometry.getPoseMeters();
  }

  /**
   * Returns the current direction of the robot.
   */
  public double getDirection() {
    if (isInverted()) {
      return 1.0;
    }

    return -1.0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    setStates(this.states);
  }
}
