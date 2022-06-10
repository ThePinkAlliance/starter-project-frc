// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ThePinkAlliance.core.joystick.Joystick;
import com.ThePinkAlliance.core.joystick.JoystickAxis;
import com.ThePinkAlliance.core.limelight.Limelight;
import com.ThePinkAlliance.core.pathweaver.PathChooser;
import com.ThePinkAlliance.core.selectable.CommandSelectable;
import com.ThePinkAlliance.core.selectable.SelectableBuilder;
import com.ctre.phoenix.motion.MotionProfileStatus;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.Drive;
import frc.robot.subsystems.Base;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final Joystick mainJS = new Joystick(0);

  private JoystickAxis x = new JoystickAxis(mainJS, Joystick.Axis.LEFT_X);

  private JoystickAxis y = new JoystickAxis(mainJS, Joystick.Axis.LEFT_Y);

  private JoystickAxis rot = new JoystickAxis(mainJS, Joystick.Axis.RIGHT_X);

  private final PathChooser m_pathChooser = new PathChooser("drivers", 2, 0);

  private final DataLog log = new DataLog("logs");

  private final CommandSelectable defaultSelectable = SelectableBuilder.build(
    "Drive Straight",
    new InstantCommand()
  );

  // The robot's subsystems and commands are defined here...
  private final Base m_base = new Base();

  private final Limelight m_limelight = new Limelight(33.3, 50);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure the dashboard for operators.
    configureDashboard();
  }

  public void configureDashboard() {
    m_pathChooser.register(defaultSelectable);
    m_pathChooser.registerDefault(defaultSelectable);
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

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Resolves the selected command that will run in autonomous
    return m_pathChooser.get();
  }
}
