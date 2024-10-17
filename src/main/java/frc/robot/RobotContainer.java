/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.commands.Angle;
import frc.robot.commands.Shoot;
import frc.robot.commands.Turn;
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
          0.15),
        m_driveTrain)
      );
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.
   * command.button.JoystickButton}.
   */
  private Trigger shooterTrigger(Trigger input) {
    Trigger active_trigger = input.negate().debounce(0.2, DebounceType.kFalling) //disables after active for 0.2 seconds
    .and(input)
    .and((new JoystickButton(m_mainStick, Constants.OIConstants.LB)).debounce(0.2)) //safety has to already be held
    .debounce(0.3, DebounceType.kFalling); //prevent spamming

    return active_trigger;
  }


  private void configureButtonBindings() {
    new JoystickButton(m_mainStick, Constants.OIConstants.Y)
      .and(new JoystickButton(m_mainStick, Constants.OIConstants.LB))
      .onTrue(new InstantCommand(() -> m_cannon.shoot(-1)));

    shooterTrigger(new JoystickButton(m_mainStick, Constants.OIConstants.X))
      .onTrue(new InstantCommand(() -> m_cannon.shoot(2)));

    shooterTrigger(new JoystickButton(m_mainStick, Constants.OIConstants.A))
      .onTrue(new InstantCommand(() -> m_cannon.shoot(3)));

    shooterTrigger(new JoystickButton(m_mainStick, Constants.OIConstants.B))
      .onTrue(new InstantCommand(() -> m_cannon.shoot(1)));

    // new JoystickButton(m_mainStick, Constants.OIConstants.X)
    //   .and(new JoystickButton(m_mainStick, Constants.OIConstants.LB))
    //   .onTrue(new InstantCommand(() -> m_cannon.shoot(2)));

    // new JoystickButton(m_mainStick, Constants.OIConstants.A)
    //   .and(new JoystickButton(m_mainStick, Constants.OIConstants.LB))
    //   .onTrue(new InstantCommand(() -> m_cannon.shoot(3)));

    // new JoystickButton(m_mainStick, Constants.OIConstants.B)
    //   .and(new JoystickButton(m_mainStick, Constants.OIConstants.LB))
    //   .onTrue(new InstantCommand(() -> m_cannon.shoot(1)));

    new JoystickButton(m_mainStick, Constants.OIConstants.RB)
      .onTrue(new Turn(m_driveTrain, 45));


    new Trigger(() -> m_mainStick.getRawAxis(5) > 0.9)
    .whileTrue(
      new Angle(m_cannon, -0.1)
    );

    new Trigger(() -> m_mainStick.getRawAxis(5) < -0.9)
    .whileTrue(
      new Angle(m_cannon, 0.1)
    );

    new JoystickButton(m_mainStick, Constants.OIConstants.START)
    .onTrue(
      new InstantCommand(() -> {m_cannon.override = true;})
    ).onFalse(
      new InstantCommand(() -> {
        m_cannon.override = false; 
        m_cannon.setValves(false, false, false);
      })
    );

    new Trigger(() -> m_mainStick.getRawAxis(5) < -0.9)
      .and(() -> m_cannon.getPitch() < -6.0)
      .debounce(5.0).onTrue(
        new InstantCommand(() -> {m_cannon.resetEncoder(-RobotConstants.CANNON_PITCH_ZERO / RobotConstants.ENCODER_TO_DEGREES);})
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
