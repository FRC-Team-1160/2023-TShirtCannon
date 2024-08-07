/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Angle;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.Cannon;
import frc.robot.subsystems.DriveTrain;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  public final DriveTrain m_driveTrain = DriveTrain.getInstance();

  private Cannon m_cannon = Cannon.getInstance();

  private Joystick m_mainStick = new Joystick(OIConstants.mainStickPort);

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    m_driveTrain.setDefaultCommand(new RunCommand(
        () -> m_driveTrain.tankDrive(
          -m_mainStick.getRawAxis(1),
          m_mainStick.getRawAxis(4),
          0.5),
        m_driveTrain)
      );
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */

  private void configureButtonBindings() {
    new JoystickButton(m_mainStick, Constants.OIConstants.Y)
      .onTrue(new InstantCommand(() -> m_cannon.shoot(-1)));

    new JoystickButton(m_mainStick, Constants.OIConstants.X)
      .onTrue(new InstantCommand(() -> m_cannon.shoot(1)));

    new JoystickButton(m_mainStick, Constants.OIConstants.A)
      .onTrue(new InstantCommand(() -> m_cannon.shoot(2)));


    new JoystickButton(m_mainStick, Constants.OIConstants.B)
      .onTrue(new InstantCommand(() -> m_cannon.shoot(3)));

      
    new JoystickButton(m_mainStick, Constants.OIConstants.LB)
    .whileTrue(
      new Angle(m_cannon, -0.1)
    );

    new JoystickButton(m_mainStick, Constants.OIConstants.RB)
    .whileTrue(
      new Angle(m_cannon, 0.1)
    );


    //whenPressed() deprecated
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
