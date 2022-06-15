package frc.robot.selectables.auto;

import com.ThePinkAlliance.core.selectable.SelectableCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

@Deprecated
public class DriveStraight implements SelectableCommand {

  @Override
  public String getName() {
    return "Drive Straight";
  }

  @Override
  public Command getCommand() {
    return new InstantCommand();
  }
}
