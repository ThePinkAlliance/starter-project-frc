package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;

public interface Container {
  public Command getAutonomousCommand();

  public void configureAdvatageKit();
}
