/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PortConstants;

public class DriveTrain extends SubsystemBase {
  /**
   * Creates a new DriveTrain.
   */

  private static DriveTrain m_instance;
  public CANSparkMax m_bL, m_mL, m_fL, m_bR, m_mR, m_fR;
  public SlewRateLimiter limiter = new SlewRateLimiter(0.8); 

  public static DriveTrain getInstance(){
		if(m_instance == null)
      m_instance = new DriveTrain();
		return m_instance;
	}
	
  public DriveTrain() {
    m_bR = new CANSparkMax(PortConstants.BACK_RIGHT, MotorType.kBrushless);
    m_mR = new CANSparkMax(PortConstants.MID_RIGHT, MotorType.kBrushless);
    m_fR = new CANSparkMax(PortConstants.FRONT_RIGHT, MotorType.kBrushless);
    m_bL = new CANSparkMax(PortConstants.BACK_LEFT, MotorType.kBrushless);
    m_fL = new CANSparkMax(PortConstants.FRONT_LEFT, MotorType.kBrushless);
    m_mL = new CANSparkMax(PortConstants.MID_LEFT, MotorType.kBrushless);
    // m_fR.setIdleMode(CANSparkBase.IdleMode.kCoast);

    SmartDashboard.putNumber("Left Tank Speed", 0);
    SmartDashboard.putNumber("Right Tank Speed", 0);
  }

  public void tankDrive(double x, double z, double speed) {
    x = limiter.calculate(x);
    if (Math.abs(x) < 0.1) x = 0;
    x *= speed;
    if (Math.abs(z) < 0.1) z = 0;
    z *= 0.25;
    
    double r = (-x+z);
    double l = (x+z);
    
    // l *= 1.5;
    
    if (Math.abs(l) > 1) l = Math.signum(l);
    if (Math.abs(r) > 1) r = Math.signum(r);
    m_bL.set(l);
    m_mL.set(l);
    m_fL.set(l);  //bad?
    
    m_bR.set(r);
    m_mR.set(r);
    m_fR.set(r);

    SmartDashboard.putNumber("Left Tank Speed", l);
    SmartDashboard.putNumber("Negative Right Tank Speed", -r);

	}

  public double getRightEncoder(){
    return m_mR.getEncoder().getPosition();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("RM encoder", getRightEncoder());
  }
}
