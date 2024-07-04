/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PortConstants;

public class Cannon extends SubsystemBase {
  /**
   * Creates a new Air.
   */
	private CANSparkMax m_pM;
	private Solenoid m_v1, m_v2, m_v3;
	double setpoint;

	PIDController pid = new PIDController(0.15, 0, 0);

	static Cannon instance;
	
	public static Cannon getInstance(){
		if(instance == null){
			instance = new Cannon();
		}
		return instance;
	}
	
	private Cannon(){
		m_v1 = new Solenoid(PneumaticsModuleType.CTREPCM, PortConstants.VALVE_1);
		//m_v2 = new Solenoid(PneumaticsModuleType.REVPH, PortConstants.VALVE_2);
		//m_v3 = new Solenoid(PneumaticsModuleType.REVPH, PortConstants.VALVE_3);
		m_pM = new CANSparkMax(PortConstants.PITCH_MOTOR, MotorType.kBrushless);
		setpoint = m_pM.getEncoder().getPosition();
		SmartDashboard.putNumber("kP", pid.getP());
	}

	public void setValves(boolean v1, boolean v2, boolean v3){
		System.out.println("Setting valve");
		m_v1.set(v1);
		//m_v2.set(v2);
		//m_v3.set(v3);
	}

	public void moveAngle(double amount){
		setpoint += amount;
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("encoder_pitch_motor", m_pM.getEncoder().getPosition());
		SmartDashboard.putNumber("setpoint", setpoint);
		
		if (setpoint - m_pM.getEncoder().getPosition() > 1) {
			setpoint = m_pM.getEncoder().getPosition();
		}
		
		m_pM.set(pid.calculate(m_pM.getEncoder().getPosition(), setpoint));
		// m_pM.set(0);
	}
}
