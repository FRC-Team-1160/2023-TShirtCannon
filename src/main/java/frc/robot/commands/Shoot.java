/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Cannon;

public class Shoot extends Command {
  /**
   * Creates a new Shoot.
   */
  private Cannon m_cannon;
  private Timer m_timer;
  private double interval;
  private int m_num;

  public Shoot(Cannon cannon, double interval) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_cannon = cannon;
    m_timer = new Timer();
    this.interval = interval;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.start();
    m_cannon.shoot(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_timer.get() >= interval){
      m_cannon.shoot(2);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_cannon.shoot(3);
    m_timer.stop();
    m_timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.get() >= (interval * 2);
  }
}
