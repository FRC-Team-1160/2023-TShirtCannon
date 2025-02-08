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

public class ConsTurn extends Turn {
  private DriveTrain m_drive;
  private PIDController pid;
  private double target;

  public ConsTurn(DriveTrain drive) {
    super(drive, 0);
  }
  
  @Override
  public void initialize() {
    pid = new PIDController(0.3, 0, 0);
    target = m_drive.getRightEncoder() + ((45.0 / 360.0) * (RobotConstants.WHEEL_BASE_WIDTH / RobotConstants.WHEEL_DIAMETER) * RobotConstants.DRIVE_GEAR_RATIO);
    pid.setSetpoint(target);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      m_drive.tankDrive(0.0, pid.calculate(m_drive.getRightEncoder()), 0.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
