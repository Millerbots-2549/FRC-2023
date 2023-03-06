// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ElevatorFeedforward;
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

  @Override
  public void periodic() {}

  public void setMotorSpeed(double speed) {
    m_elevatorMotor.set(speed);
  }

  public void setMotorVolts(double volts, double velocity) {
    double feedforward = m_feedforward.calculate(velocity);
    m_elevatorMotor.setVoltage(volts+feedforward);
  }

  public double getEncoderDistance() {
    return m_elevatorEncoder.getPosition();
  }

  public double getEncoderVelocity() {
    return m_elevatorEncoder.getVelocity();
  }
}
