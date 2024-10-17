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
    addRequirements(drive);
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    m_angle = angle;
    
  }
  //joe mama
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid = new PIDController(0.3, 0, 0);
    filter = new SlewRateLimiter(1);
    SmartDashboard.putString("Test2", "Started");
    target = m_drive.getMiddleEncoder() + ((m_angle / 360.0) * (RobotConstants.WHEEL_BASE_WIDTH / RobotConstants.WHEEL_DIAMETER) * RobotConstants.DRIVE_GEAR_RATIO);
    pid.setSetpoint(target);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double z = (pid.calculate(m_drive.getMiddleEncoder()));
    if (Math.abs(z) > 1) z = 1 * Math.signum(z);
    z = filter.calculate(z);
    m_drive.tankDrive(0.0, z, 0);
    SmartDashboard.putNumber("Turn Command Setpoint", target);
    SmartDashboard.putNumber("Turn Command PID", pid.calculate(m_drive.getMiddleEncoder()));
    SmartDashboard.putString("Test2", "Running");


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //m_timer.stop();
    //m_timer.reset();
    SmartDashboard.putString("Test2", "End");

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putNumber("Test1", pid.getPositionError());
    return Math.abs(pid.getPositionError()) < (3.0 / 360.0) * RobotConstants.DRIVE_GEAR_RATIO;//m_timer.get() >= 0.05;
  }
}
