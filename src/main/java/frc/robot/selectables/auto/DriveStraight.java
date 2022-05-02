package frc.robot.selectables.auto;

import com.ThePinkAlliance.core.selectable.CommandSelectable;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class DriveStraight implements CommandSelectable {

  @Override
  public String getName() {
    return "Drive Straight";
  }

  @Override
  public Command getCommand() {
    return new InstantCommand();
  }
}
