// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TankBase extends SubsystemBase {
  DifferentialDrive differentialDrive;
  MotorControllerGroup leftControllerGroup;
  MotorControllerGroup rightControllerGroup;

  /** Creates a new TankBase. */
  public TankBase() {
    leftControllerGroup = new MotorControllerGroup(new WPI_TalonFX(0), new WPI_TalonFX(1));
    rightControllerGroup = new MotorControllerGroup(new WPI_TalonFX(2), new WPI_TalonFX(3));

    differentialDrive = new DifferentialDrive(leftControllerGroup, rightControllerGroup);
  }

  public void configureDeadband(double zone) {
    this.differentialDrive.setDeadband(zone);
  }

  public void drive(double left, double right) {
    this.differentialDrive.tankDrive(left, right, true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
