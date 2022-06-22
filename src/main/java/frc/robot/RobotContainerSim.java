// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ThePinkAlliance.core.joystick.Joystick;
import com.ThePinkAlliance.core.joystick.JoystickAxis;
import com.ThePinkAlliance.core.limelight.Limelight;
import com.ThePinkAlliance.core.pathweaver.PathChooser;
import com.ThePinkAlliance.core.selectable.SelectableTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Drive;
import frc.robot.commands.MotionProfileTest;
import frc.robot.subsystems.Base;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedNetworkTables;
import org.littletonrobotics.junction.io.ByteLogReceiver;
import org.littletonrobotics.junction.io.ByteLogReplay;
import org.littletonrobotics.junction.io.LogSocketServer;

/** Add your docs here. */
public class RobotContainerSim implements Container {

  private final Joystick mainJS = new Joystick(0);

  private JoystickAxis x = new JoystickAxis(mainJS, Joystick.Axis.LEFT_X);

  private JoystickAxis y = new JoystickAxis(mainJS, Joystick.Axis.LEFT_Y);

  private JoystickAxis rot = new JoystickAxis(mainJS, Joystick.Axis.RIGHT_X);

  private final PathChooser m_pathChooser = new PathChooser("drivers", 2, 0);

  // The robot's subsystems and commands are defined here...
  private final Base m_base = new Base();

  // Make sure to calibrate the limelight Crosshairs before using it.
  private final Limelight m_limelight = new Limelight(33.3, 50);

  private final SelectableTrajectory trajectory = new SelectableTrajectory(
    "straight",
    "straight"
  );

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainerSim() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure the dashboard for operators.
    configureDashboard();
  }

  public void configureDashboard() {
    m_pathChooser.registerDefault(trajectory);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    m_base.setDefaultCommand(new Drive(m_base, x, y, rot));
  }

  @Override
  public void configureAdvatageKit() {
    Logger logger = Logger.getInstance();

    LoggedNetworkTables.getInstance().addTable("/SmartDashboard"); // Log & replay "SmartDashboard" values (no tables are logged by default).
    LoggedNetworkTables.getInstance().addTable("/debug"); // Log & replay "SmartDashboard" values (no tables are logged by default).
    logger.recordMetadata("ProjectName", Version.GIT_SHA); // Set a metadata value

    logger.addDataReceiver(new LogSocketServer(5800));
    String path =
      "C:\\Users\\%USERNAME%\\Desktop\\code\\FRC\\Starter-Project-FRC\\logs\\test.rlog"; // Prompt the user for a file path on the command line
    Logger.getInstance().setReplaySource(new ByteLogReplay(path)); // Read log file for replay
    Logger
      .getInstance()
      .addDataReceiver(
        new ByteLogReceiver(ByteLogReceiver.addPathSuffix(path, "_sim"))
      ); // Save replay results to a new log with the "_sim" suffix

    logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  @Override
  public Command getAutonomousCommand() {
    // Resolves the selected command that will run in autonomous

    return new MotionProfileTest();
  }
}
