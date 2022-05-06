// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ThePinkAlliance.core.joystick.Joystick;
import com.ThePinkAlliance.core.joystick.JoystickAxis;
import com.ThePinkAlliance.core.pathweaver.PathChooser;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.Drive;
import frc.robot.selectables.auto.DriveStraight;
import frc.robot.subsystems.Base;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final Joystick mainJS = new Joystick(0);

  private JoystickAxis x = new JoystickAxis(mainJS, Joystick.Axis.LEFT_X)
    .withAxisCubed()
    .withDeadband()
    .withLimit(55);

  private JoystickAxis y = new JoystickAxis(mainJS, Joystick.Axis.LEFT_Y)
    .withAxisCubed()
    .withDeadband()
    .withLimit(55);

  private JoystickAxis rot = new JoystickAxis(mainJS, Joystick.Axis.RIGHT_X)
    .withAxisCubed()
    .withDeadband()
    .withLimit(55);

  private final PathChooser m_pathChooser = new PathChooser("drivers", 2, 0);

  // The robot's subsystems and commands are defined here...
  private final Base m_base = new Base();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure the dashboard for operators.
    configureDashboard();
  }

  public void configureDashboard() {
    m_pathChooser.register(new DriveStraight());
    m_pathChooser.registerDefault(new DriveStraight());
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
    // An ExampleCommand will run in autonomous
    return new InstantCommand();
  }
}
