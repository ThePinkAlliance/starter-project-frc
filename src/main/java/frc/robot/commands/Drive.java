// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ThePinkAlliance.core.joystick.JoystickAxis;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Base;

public class Drive extends CommandBase {

  Base m_base;

  JoystickAxis x;
  JoystickAxis y;
  JoystickAxis rot;

  /** Creates a new Drive. */
  public Drive(Base m_base, JoystickAxis x, JoystickAxis y, JoystickAxis rot) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_base = m_base;

    this.x = x;
    this.y = y;
    this.rot = rot;

    addRequirements(m_base);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double x = this.x.get();
    double y = this.y.get();
    double rot = this.rot.get();

    SmartDashboard.putNumber("x", x);
    SmartDashboard.putNumber("y", y);
    SmartDashboard.putNumber("rot", rot);

    m_base.drive(new ChassisSpeeds(x, y, rot));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_base.drive(new ChassisSpeeds());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
