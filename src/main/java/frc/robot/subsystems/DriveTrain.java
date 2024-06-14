/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PortConstants;

public class DriveTrain extends SubsystemBase {
  /**
   * Creates a new DriveTrain.
   */
  private static DriveTrain m_instance;
  private CANSparkMax m_bL, m_mL, m_fL, m_bR, m_mR, m_fR;

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
  }

  public void tankDrive(double x, double z, double speed) {
    if (Math.abs(x) < 0.1) x = 0;
    double r = speed * (-x+z);
    double l = speed * (x+z);
    m_bL.set(l);
    m_mL.set(l);
    m_fL.set(l);
    
    m_bR.set(r);
    m_mR.set(r);
    m_fR.set(r);

	}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
