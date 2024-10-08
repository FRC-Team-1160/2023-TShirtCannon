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
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.RobotConstants;

public class Turn extends Command {
  /**
   * Creates a new Angle.
   */
  private DriveTrain m_drive;
  private double m_angle;
  private PIDController pid;
  private SlewRateLimiter filter;
  private double target;

  public Turn(DriveTrain drive, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    m_angle = angle;
    pid = new PIDController(0.1, 0, 0);
    filter = new SlewRateLimiter(0.5);
    target = m_drive.m_mR.getEncoder().getPosition() + angle * (RobotConstants.WHEEL_BASE_WIDTH / RobotConstants.WHEEL_DIAMETER) / RobotConstants.ENCODER_TO_DEGREES;
    pid.setSetpoint(target);
  }
  //joe mama
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.tankDrive(0, filter.calculate(pid.calculate(m_drive.getMiddleEncoder())), 0.25);
    SmartDashboard.putNumber("Turn Command PID", pid.calculate(m_drive.getMiddleEncoder()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //m_timer.stop();
    //m_timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return Math.abs(pid.getPositionError()) < 10 / RobotConstants.ENCODER_TO_DEGREES;//m_timer.get() >= 0.05;
  }
}
