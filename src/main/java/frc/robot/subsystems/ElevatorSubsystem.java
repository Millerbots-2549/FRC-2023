// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ManipulatorConstants.*;

public class ElevatorSubsystem extends SubsystemBase {
  private final CANSparkMax m_elevatorMotor;
  private final RelativeEncoder m_elevatorEncoder;
  
  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    m_elevatorMotor = new CANSparkMax(kElevatorMotorPort, MotorType.kBrushless);
    m_elevatorEncoder = m_elevatorMotor.getEncoder();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setElevatorMotorSpeed(double speed) {
    m_elevatorMotor.set(speed);
    SmartDashboard.putNumber("elevator encoder", m_elevatorMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("elevator current", m_elevatorMotor.getOutputCurrent());
  }

  public double getElevatorMotorCurrent() {
    return m_elevatorMotor.getOutputCurrent();
  }

  public double getPosition() {
    return m_elevatorEncoder.getPosition();
  }

  public void resetEncoder() {
    // Zeroes the encoder on the elevator motor
    m_elevatorMotor.getEncoder().setPosition(0);
  }
}
