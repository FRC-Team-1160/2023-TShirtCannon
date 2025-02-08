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
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.PortConstants;

/**
 * The Subsystem class representing the cannon.
 */
public class Cannon extends SubsystemBase {
	public CANSparkMax m_pM;
	public Solenoid m_v1, m_v2, m_v3; // TODO: switch to low, med, high

	double setpoint;
	double zero;
	public boolean override;

	PIDController pid = new PIDController(0.15, 0, 0);

	static Cannon instance;
	
	public static Cannon getInstance(){
		if(instance == null){
			instance = new Cannon();
		}
		return instance;
	}
	
	private Cannon(){

		m_v1 = new Solenoid(PortConstants.PCM, PneumaticsModuleType.CTREPCM, PortConstants.VALVE_1); // TODO: switch to low, med, high
		m_v2 = new Solenoid(PortConstants.PCM, PneumaticsModuleType.CTREPCM, PortConstants.VALVE_2); // TODO: switch to low, med, high
		m_v3 = new Solenoid(PortConstants.PCM, PneumaticsModuleType.CTREPCM, PortConstants.VALVE_3); // TODO: switch to low, med, high

		m_v1.setPulseDuration(RobotConstants.PULSE_DURATION);
		m_v2.setPulseDuration(RobotConstants.PULSE_DURATION);
		m_v3.setPulseDuration(RobotConstants.PULSE_DURATION);

		m_pM = new CANSparkMax(PortConstants.PITCH_MOTOR, MotorType.kBrushless);

		zero = setPitch();

	}

	/**
	 * Actuate valves based on boolean input
	 * @param v1 State of valve 1
	 * @param v2 State of valve 2
	 * @param v3 State of valve 3
	 */
	public void setValves(boolean v1, boolean v2, boolean v3){
		System.out.println("Setting valve");
		m_v1.set(v1);
		m_v2.set(v2);
		m_v3.set(v3);

		
		
	}	


	/**
	 * Shoots the cannons based on the boolean parameters.
	 * @param v1
	 * @param v2
	 * @param v3
	 */
	public void shoot(boolean v1, boolean v2, boolean v3){		
		if (v1) m_v1.startPulse();
		if (v2) m_v2.startPulse();
		if (v3) m_v3.startPulse();
		System.out.println("shoot");
	}

	/**
	 * Manual Override: Shoots one cannon or all, based on an integer; Shoots all on -1.
	 * @param num
	 */
	public void shoot(int num){
		if (!override){
			switch(num) {
				case 1:
					m_v1.startPulse();
					break;
				case 2:
					m_v2.startPulse();
					break;
				case 3:
					m_v3.startPulse();
					break;
				case -1:
				m_v1.startPulse();
				m_v2.startPulse();
				m_v3.startPulse();
			}
			System.out.println("Shoot" + num);
		}

	}

	/**
	 * Changes the vertical angle of the cannons based on a double.
	 * @param amount
	 */
	public void moveAngle(double amount){
		setpoint += amount;
	}

	/**
	 * Runs when the robot starts to set setpoint to the current angle; This prevents the robot from attempting to return to the angle it was set to before it was turned off.
	 * @return
	 */
	public double setPitch(){
		setpoint = m_pM.getEncoder().getPosition();
		return setpoint;
	}

	/**
	 * Sets the vertical angle of the cannons based on a double parameter.
	 * @param setpoint
	 */
	public void resetEncoder(double setpoint){
		m_pM.getEncoder().setPosition(setpoint);
	}

	/**
	 * Returns the vertical angle of the cannon.
	 * @return
	 */
	public double getPitch(){
		return m_pM.getEncoder().getPosition();
	}

	/**
	 * Runs constantly, checks if the cannon related button pressed down and runs the relevant functions; Checks cannon angle.
	 */

	@Override
	public void periodic() {
		if (override){
		  setValves(
			SmartDashboard.getBoolean("Valve 1", false),
			SmartDashboard.getBoolean("Valve 2", false),
			SmartDashboard.getBoolean("Valve 3", false));
		} else {
		  SmartDashboard.putBoolean("Valve 1", m_v1.get());
		  SmartDashboard.putBoolean("Valve 2", m_v2.get());
		  SmartDashboard.putBoolean("Valve 3", m_v3.get());
		}
		double encoder_pos = m_pM.getEncoder().getPosition();
		SmartDashboard.putNumber("encoder_pitch_motor", encoder_pos);
		SmartDashboard.putNumber("Cannon Pitch", RobotConstants.CANNON_PITCH_ZERO + (encoder_pos) * RobotConstants.ENCODER_TO_DEGREES); // CALIBRATE
		SmartDashboard.putNumber("setpoint", setpoint);
		SmartDashboard.putBoolean("Cannon Override", override);
		
		if (Math.abs(setpoint - m_pM.getEncoder().getPosition()) > 1) {
			setPitch();
		}
		
		m_pM.set(pid.calculate(m_pM.getEncoder().getPosition(), setpoint));
		// m_pM.set(0);

	}
}
