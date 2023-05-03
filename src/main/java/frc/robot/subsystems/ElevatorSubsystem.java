// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ManipulatorConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ElevatorSubsystem extends SubsystemBase {
  private final CANSparkMax m_elevatorMotor = 
    new CANSparkMax(kElevatorMotorPort, MotorType.kBrushless);
  private final RelativeEncoder m_elevatorEncoder = 
    m_elevatorMotor.getEncoder();
  private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(kElevatorFeedforwardKS, kElevatorFeedforwardKG, kElevatorFeedforwardKV, kElevatorFeedforwardKA);

  public ElevatorSubsystem() {
    m_elevatorEncoder.setPositionConversionFactor(kElevatorDistancePerPulse);
    m_elevatorMotor.setInverted(true);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("elevator encoder", m_elevatorEncoder.getPosition());
    if(m_elevatorEncoder.getPosition() > 0.80){
      m_elevatorMotor.setSmartCurrentLimit(5);
    }else{
      m_elevatorMotor.setSmartCurrentLimit(20);
    }
  }

  public void setMotorSpeed(double speed) {
    double m_adjustedDeadzoneSpeed = 0.0;
    if (Math.abs(speed) > kElevatorJoystickDeadzone)
      m_adjustedDeadzoneSpeed = speed - (Math.signum(speed) * kElevatorJoystickDeadzone);
    if (m_elevatorEncoder.getPosition() > kElevatorHighestPosition && -m_adjustedDeadzoneSpeed > 0)
      m_elevatorMotor.set(0.0);
    else
      m_elevatorMotor.set(-m_adjustedDeadzoneSpeed);
  }

  public void setMotorVolts(double volts, double velocity) {
    double feedforward = m_feedforward.calculate(velocity);
    if (m_elevatorEncoder.getPosition() > kElevatorHighestPosition && volts+feedforward > 0.0)
      m_elevatorMotor.set(0.0);
    else
      m_elevatorMotor.setVoltage(volts+feedforward);
  }

  public double getEncoderDistance() {
    return m_elevatorEncoder.getPosition();
  }

  public double getEncoderVelocity() {
    return m_elevatorEncoder.getVelocity();
  }

  public void resetEncoder() {
    m_elevatorEncoder.setPosition(kElevatorLowNodePosition);
  }
}
