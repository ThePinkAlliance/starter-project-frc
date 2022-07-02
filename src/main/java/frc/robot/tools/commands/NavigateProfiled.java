// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.tools.commands;

import com.ThePinkAlliance.swervelib.SdsModuleConfigurations;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.ThePinkAlliance.core.ctre.talon.TalonUtils;
import com.ThePinkAlliance.core.util.Gains;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.Base;

public class NavigateProfiled extends CommandBase {

  TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(Constants.MAX_VELOCITY_METERS_PER_SECOND,
      Constants.MAX_ACCELERATION_METERS_PER_SECOND);

  TrapezoidProfile.State idle_state = new TrapezoidProfile.State();

  Base base;

  Gains drive_gains = new Gains(1, 0.5, 0.002);
  Gains theta_gains = new Gains(6.0, 0, 0);

  double straight_tolerance = 3;
  double align_tolerance = 1;

  boolean bBackwards = false;
  boolean debug = false;

  double TRACKER_LIMIT_DEFAULT = 0.45;

  /**
   * kP:
   * kI:
   * kD: keep kD low otherwise your system could become unstable
   */

  ProfiledPIDController straightController = new ProfiledPIDController(
      drive_gains.kP,
      drive_gains.kI,
      drive_gains.kD, constraints); // kP 0.27 kI 0.3 kD 0.002

  PIDController alignController = new PIDController(
      theta_gains.kP,
      theta_gains.kI,
      theta_gains.kD);

  double reduction = SdsModuleConfigurations.MK4I_L1.getDriveReduction();

  double targetAngle = 0;
  double targetInches = 0;

  /** Creates a new DriveStraight. */
  public NavigateProfiled(Base base, double targetInches, double targetAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.base = base;
    this.targetInches = targetInches;
    this.targetAngle = targetAngle;
    addRequirements(base);
  }

  public NavigateProfiled(Base base, double targetInches) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.base = base;
    this.targetInches = targetInches;
    addRequirements(base);
  }

  public NavigateProfiled(Base base, double targetInches, boolean bBackwards) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.base = base;
    this.targetInches = targetInches;
    this.bBackwards = bBackwards;
    addRequirements(base);
  }

  public NavigateProfiled(
      Base base,
      double targetInches,
      double targetAngle,
      boolean bBackwards) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.base = base;
    this.targetInches = targetInches;
    this.targetAngle = targetAngle;
    this.bBackwards = bBackwards;
    addRequirements(base);
  }

  public CommandBase configureTolerance(double tolerance) {
    this.straight_tolerance = tolerance;

    return this;
  }

  public CommandBase configureGains(Gains drive) {
    this.drive_gains = drive;

    return this;
  }

  public CommandBase configureGains(Gains drive, Gains theta) {
    this.drive_gains = drive;
    this.theta_gains = theta;

    return this;
  }

  /**
   * Motion profile constrants are only applyed to the straight controller.
   * 
   * @param constraints Can be created by using
   *                    <a href=
   *                    "https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/trapezoidal-profiles.html#constraints">TrapezoidProfile.Constraints</a>
   */
  public CommandBase configureProfileConstrants(TrapezoidProfile.Constraints constraints) {
    this.constraints = constraints;

    return this;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    base.drive(new ChassisSpeeds());
    alignController.reset();
    alignController.enableContinuousInput(-180.0, 180.0);
    alignController.setTolerance(this.align_tolerance);
    base.zeroGyro();

    straightController.reset(this.idle_state);
    straightController.setTolerance(this.straight_tolerance);
    base.resetDriveMotors();

    this.straightController.setConstraints(constraints);

    if (debug) {
      straightController.setP(
          SmartDashboard.getNumber("NAVIGATE-DRIVE-KP", drive_gains.kP));
      straightController.setI(
          SmartDashboard.getNumber("NAVIGATE-DRIVE-KI", drive_gains.kI));
      straightController.setD(
          SmartDashboard.getNumber("NAVIGATE-DRIVE-KD", drive_gains.kD));
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double x_output = 0.0;
    double x_power = 0.0;
    double turnPower = 0.0;

    // Drive
    if (targetInches != 0) {
      // double front_left_pos = Math.abs(
      // this.base.frontLeftModule.getDrivePosition()
      // );
      double front_right_pos = Math.abs(
          this.base.frontRightModule.getDrivePosition());

      // 0.123825 is the swerve pod drive reduction.
      double distance_traveled_inches = ((0.123825) * (front_right_pos / TalonUtils.FULL_TALON_ROTATION_TICKS)) *
          12.875;

      x_output = straightController.calculate(distance_traveled_inches, targetInches);
      x_power = (x_output / targetInches) * Constants.MAX_VELOCITY_METERS_PER_SECOND;

      if (debug) {
        System.out.println("Navigate: " + x_power + ", Output" + x_output);
        SmartDashboard.putNumber("traveled", distance_traveled_inches);
      }
    }
    // Turn: PID Controller using setpoint of zero
    else if (targetAngle != 0) {
      double currentAngle = base.getSensorYaw();
      double processVariable = Math.abs(targetAngle) - Math.abs(currentAngle);
      processVariable = Math.copySign(processVariable, targetAngle);
      double output = alignController.calculate(processVariable, 0);
      double limitedTurnPower = limitPower(output / 180, TRACKER_LIMIT_DEFAULT);
      turnPower = limitedTurnPower * Constants.MAX_VELOCITY_METERS_PER_SECOND;

      if (debug) {
        SmartDashboard.putNumber("Navigate Output: ", output);
        SmartDashboard.putNumber("Navigate Turn Power:", turnPower);
        SmartDashboard.putNumber("Navigate Limited Power:", limitedTurnPower);
        SmartDashboard.putNumber("Navigate Current Angle:", currentAngle);
        SmartDashboard.putNumber("Navigate Process Variable:", processVariable);
      }

    }

    if (bBackwards)
      x_power *= -1;
    ChassisSpeeds speeds = new ChassisSpeeds(x_power, 0, turnPower);
    base.drive(speeds);
  }

  private double limitPower(double currentPower, double limit) {
    double value = currentPower;
    if (Math.abs(currentPower) > limit)
      value = Math.copySign(limit, currentPower);

    if (debug)
      System.out.println("limitPower: " + value + "; Original: " + currentPower);

    return value;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    base.drive(new ChassisSpeeds(0, 0, 0));
    base.resetDriveMotors();

    System.out.println(
        "END OF COMMAND: " +
            this.base.frontRightModule.getDrivePosition() +
            ", " +
            ((SdsModuleConfigurations.MK4I_L1.getDriveReduction()) *
                (this.base.frontRightModule.getDrivePosition() /
                    TalonUtils.FULL_TALON_ROTATION_TICKS)
                *
                Base.DRIVE_WHEEL_CIRCUMFERENCE)
            +
            ", " +
            "INTERRUPTED: " +
            interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean straightMet = straightController.atSetpoint();
    boolean turnMet = alignController.atSetpoint();
    System.out.println(
        "Straight Met: " + straightMet + "; turnMet: " + turnMet);

    // ONLY CHECK THE CONDITION FOR THE MOVEMENT WHOSE TARGET IS NOT ZERO
    if (targetInches == 0)
      return turnMet;
    else
      return straightMet;
  }
}
