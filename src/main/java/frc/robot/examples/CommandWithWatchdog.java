// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.examples;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class CommandWithWatchdog extends CommandBase {
  Watchdog watchdog;

  Timer timer;

  boolean isFinished = false;

  /** Creates a new CommandWithWatchdog. */
  public CommandWithWatchdog() {
    // Use addRequirements() here to declare subsystem dependencies.

    this.watchdog = new Watchdog(9, () -> {
      // Insert callback here.
    });
    this.timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
    watchdog.enable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.hasElapsed(10)) {
      isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    watchdog.disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished || watchdog.isExpired();
  }
}
