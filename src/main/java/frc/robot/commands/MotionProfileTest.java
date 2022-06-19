// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ThePinkAlliance.core.rev.RevNeo550;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.REVPhysicsSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.simulator.PhysicsSim;

public class MotionProfileTest extends CommandBase {

  // The WPI talons allow us to view motor data in the sim.
  WPI_TalonFX talon = new WPI_TalonFX(0);
  TalonFXSimCollection collection = talon.getSimCollection();

  RevNeo550 neo550 = new RevNeo550(0);

  /** Creates a new MotionProfileTest. */
  public MotionProfileTest() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    talon.configSelectedFeedbackSensor(
      TalonFXFeedbackDevice.IntegratedSensor,
      0,
      30
    );
    talon.config_kP(0, 0.1);
    // talon.config_kI(0, 0.002);

    PhysicsSim.getInstance().addTalonFX(talon, 3, 20660);
    REVPhysicsSim.getInstance().addSparkMax(neo550, DCMotor.getNeo550(1));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    talon.set(ControlMode.Position, 1000);
    neo550.set(1);

    SmartDashboard.putNumber("motor", talon.getSelectedSensorPosition());
    SmartDashboard.putNumber("motor 2", neo550.get());
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
