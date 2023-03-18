// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ManipulatorConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ArmSubsystem extends SubsystemBase {
  private final CANSparkMax m_armMotor = new CANSparkMax(kArmMotorPort, MotorType.kBrushed);
  private final Encoder m_armEncoder = new Encoder(kArmEncoderPorts[0], kArmEncoderPorts[1]);
  private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(kArmFeedforwardKS, kArmFeedforwardKG, kArmFeedforwardKV, kArmFeedforwardKA);

  public ArmSubsystem() {
    m_armEncoder.setDistancePerPulse(kArmEncoderDistancePerPulseMeters);
    m_armEncoder.setReverseDirection(true);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("arm encoder", m_armEncoder.getDistance());
  }

  public void setMotorSpeed(double speed) {
    m_armMotor.set(speed);
  }

  public void setMotorVolts(double volts, double velocity) {
    double feedforward = m_feedforward.calculate(velocity);
    m_armMotor.setVoltage(volts+feedforward);
    SmartDashboard.putNumber("arm rate", m_armEncoder.getRate());
    SmartDashboard.putNumber("arm volts", volts);
    SmartDashboard.putNumber("arm velocity", velocity);
    SmartDashboard.putNumber("feedforward", feedforward);
  }

  public double getEncoderDistance() {
    return m_armEncoder.getDistance();
  }

  //public double getEncoderVelocity() {
  //  return m_armEncoder.getVelocity();
  //}
}
