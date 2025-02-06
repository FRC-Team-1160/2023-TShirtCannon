/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Cannon;
//import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.math.filter.SlewRateLimiter;

public class Angle extends Command {
  /**
   * Creates a new Angle.
   */
  private Cannon m_cannon;
  private double m_amount;
  private SlewRateLimiter filter;

  public Angle(Cannon cannon, double amount) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_cannon = cannon;
    m_amount = amount;
    filter = new SlewRateLimiter(0.3);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_cannon.moveAngle(filter.calculate(m_amount));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_cannon.moveAngle(filter.calculate(m_amount));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //m_timer.stop();
    m_cannon.moveAngle(0);
    //m_timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;//m_timer.get() >= 0.05;
  }
}
