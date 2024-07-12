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
	public CANSparkMax m_pM;
	public Solenoid m_v1, m_v2, m_v3;

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
		m_v1 = new Solenoid(PortConstants.PCM, PneumaticsModuleType.CTREPCM, PortConstants.VALVE_1);
		m_v2 = new Solenoid(PortConstants.PCM, PneumaticsModuleType.CTREPCM, PortConstants.VALVE_2);
		m_v3 = new Solenoid(PortConstants.PCM, PneumaticsModuleType.CTREPCM, PortConstants.VALVE_3);

		m_v1.setPulseDuration(0.5);
		m_v2.setPulseDuration(0.5);
		m_v3.setPulseDuration(0.5);

		m_pM = new CANSparkMax(PortConstants.PITCH_MOTOR, MotorType.kBrushless);
		setpoint = m_pM.getEncoder().getPosition();
	}

	public void setValves(boolean v1, boolean v2, boolean v3){
		System.out.println("Setting valve");
		m_v1.set(v1);
		//m_v2.set(v2);
		//m_v3.set(v3);
	}

	public void shoot(boolean v1, boolean v2, boolean v3){
		if (v1) m_v1.startPulse();
		if (v2) m_v2.startPulse();
		if (v3) m_v3.startPulse();
	}

	public void shoot(int num){
		switch(num) {
			case 1: m_v1.startPulse();
			break;
			case 2: m_v2.startPulse();
			break;
			case 3: m_v3.startPulse();
			break;
			case -1:
			m_v1.startPulse();
			m_v2.startPulse();
			m_v3.startPulse();
		}

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
