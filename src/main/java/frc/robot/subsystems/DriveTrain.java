/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PortConstants;

public class DriveTrain extends SubsystemBase {
  /**
   * Creates a new DriveTrain.
   */
  private static DriveTrain m_instance;
  private CANSparkMax m_bL, m_mL, m_fL, m_bR, m_mR, m_fR;
  
  private MotorController m_l, m_r;

  private DifferentialDrive m_drive;

  public static DriveTrain getInstance(){
		if(m_instance == null)
      m_instance = new DriveTrain();
		return m_instance;
	}
	
  public DriveTrain() {
    m_bL = new CANSparkMax(PortConstants.BACK_LEFT, MotorType.kBrushless);
    m_mR = new CANSparkMax(PortConstants.MID_RIGHT, MotorType.kBrushless);
    m_bR = new CANSparkMax(PortConstants.BACK_RIGHT, MotorType.kBrushless);
    m_fL = new CANSparkMax(PortConstants.FRONT_LEFT, MotorType.kBrushless);
    m_mL = new CANSparkMax(PortConstants.MID_LEFT, MotorType.kBrushless);
    m_fR = new CANSparkMax(PortConstants.FRONT_RIGHT, MotorType.kBrushless);

    m_l = new MotorControllerGroup(m_bL, m_mL, m_fL);
    m_r = new MotorControllerGroup(m_bR, m_mR, m_fR);
  
    m_drive = new DifferentialDrive(m_l, m_r);
  
  }

  public void tankDrive(double x, double z, double correction){
    System.out.println("x" + String.valueOf(-x+z) + "z" + String.valueOf(-x-z));
    m_drive.tankDrive(-x-z, x-z);
	}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
