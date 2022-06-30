// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ThePinkAlliance.swervelib.Mk4SwerveModuleHelper;
import com.ThePinkAlliance.swervelib.Mk4iSwerveModuleHelper;
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

  public static final double DRIVE_WHEEL_CIRCUMFERENCE = 12.875;

  public SwerveModule frontLeftModule;
  public SwerveModule frontRightModule;
  public SwerveModule backLeftModule;
  public SwerveModule backRightModule;

  AHRS gyro;

  ChassisSpeeds chassisSpeeds;
  SwerveDriveKinematics kinematics;
  SwerveDriveOdometry odometry;
  SwerveModuleState[] states;

  ShuffleboardTab tab = Shuffleboard.getTab("debug");

  /** Creates a new Base. */
  public Base() {
    this.chassisSpeeds = new ChassisSpeeds();

    this.gyro = new AHRS();

    this.kinematics = new SwerveDriveKinematics(
        // Front Left Pod
        new Translation2d(
            Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
            Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
        // Front Right
        new Translation2d(
            Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
            -Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
        // Back Left
        new Translation2d(
            -Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
            Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
        // Back Right
        new Translation2d(
            -Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
            -Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0));

    this.odometry = new SwerveDriveOdometry(
        kinematics,
        Rotation2d.fromDegrees(gyro.getFusedHeading()));

    this.states = kinematics.toSwerveModuleStates(new ChassisSpeeds());

    this.configureMk4(Constants.gearRatio);
    this.configurePods();
  }

  public void configureMk4(Mk4SwerveModuleHelper.GearRatio ratio) {
    this.frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(
        tab
            .getLayout("Front Left Module", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(0, 0),
        ratio,
        Constants.frontLeftConfig);

    this.frontRightModule = Mk4SwerveModuleHelper.createFalcon500(
        tab
            .getLayout("Front Right Module", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(0, 2),
        ratio,
        Constants.frontRightConfig);

    this.backRightModule = Mk4SwerveModuleHelper.createFalcon500(
        tab
            .getLayout("Back Right Module", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(0, 4),
        ratio,
        Constants.backRightConfig);

    this.backLeftModule = Mk4SwerveModuleHelper.createFalcon500(
        tab
            .getLayout("Back Left Module", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(0, 6),
        ratio,
        Constants.backLeftConfig);
  }

  public void configureMk4i(Mk4iSwerveModuleHelper.GearRatio ratio) {
    this.frontLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
        tab
            .getLayout("Front Left Module", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(0, 0),
        ratio,
        Constants.frontLeftConfig);

    this.frontRightModule = Mk4iSwerveModuleHelper.createFalcon500(
        tab
            .getLayout("Front Right Module", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(0, 2),
        ratio,
        Constants.frontRightConfig);

    this.backRightModule = Mk4iSwerveModuleHelper.createFalcon500(
        tab
            .getLayout("Back Right Module", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(0, 4),
        ratio,
        Constants.backRightConfig);

    this.backLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
        tab
            .getLayout("Back Left Module", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(0, 6),
        ratio,
        Constants.backLeftConfig);
  }

  /**
   * Takes ChassisSpeed object and converts it to swerve module states to send to
   * all the modules.
   *
   * @param speeds
   */
  public void drive(ChassisSpeeds speeds) {
    states = kinematics.toSwerveModuleStates(speeds);
  }

  /**
   * Set's the current states for all the Swerve modules to the desired one's.
   *
   * @param states swerve pod states
   */
  public void setStates(SwerveModuleState... states) {
    odometry.update(gyro.getRotation2d(), states);

    SwerveDriveKinematics.desaturateWheelSpeeds(
        states,
        Constants.MAX_VELOCITY_METERS_PER_SECOND);

    frontLeftModule.set(
        convertModuleSpeed(states[0].speedMetersPerSecond),
        states[0].angle.getRadians());

    frontRightModule.set(
        convertModuleSpeed(states[1].speedMetersPerSecond),
        states[1].angle.getRadians());

    backLeftModule.set(
        convertModuleSpeed(states[2].speedMetersPerSecond),
        states[2].angle.getRadians());

    backRightModule.set(
        convertModuleSpeed(states[3].speedMetersPerSecond),
        states[3].angle.getRadians());
  }

  /**
   * Converts module speed in meters per second to desired motor voltage.
   *
   * @param speedMetersPerSecond
   */
  public double convertModuleSpeed(double speedMetersPerSecond) {
    return ((speedMetersPerSecond / Constants.MAX_VELOCITY_METERS_PER_SECOND) *
        Constants.MAX_VOLTAGE);
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
    return (getRotation().getDegrees() <= 190 &&
        getRotation().getDegrees() > 90 ||
        getRotation().getDegrees() >= 290 &&
            getRotation().getDegrees() < 90);
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
   * Calculates the desired angle relative to the robot's heading.
   * 
   * @param desiredAngle The desired rotation in degress
   * @return The angle in degress the drivetrain needs to move.
   */
  public Rotation2d calculateDesiredAngle(Rotation2d desiredAngle) {
    return new Rotation2d(this.getSensorYaw() - desiredAngle.getDegrees());
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

  /**
   * Reset the encoder counts on all the pod drive motors.
   */
  public void resetDriveMotors() {
    this.backLeftModule.resetDrive();
    this.backRightModule.resetDrive();
    this.frontLeftModule.resetDrive();
    this.frontRightModule.resetDrive();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    setStates(this.states);
  }

  private void configurePods() {
    if (this.frontLeftModule == null || this.frontRightModule == null || this.backLeftModule == null
        || this.backRightModule == null) {
      throw new Error("Please configure the swerve pod type");
    }

    this.frontLeftModule.configRampRate(
        Constants.frontLeftConfig.getDriveRampRate());
    this.frontRightModule.configRampRate(
        Constants.frontRightConfig.getDriveRampRate());

    this.backLeftModule.configRampRate(
        Constants.backLeftConfig.getDriveRampRate());
    this.backRightModule.configRampRate(
        Constants.backRightConfig.getDriveRampRate());
  }
}
