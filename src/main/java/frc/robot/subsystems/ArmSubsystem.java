// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ManipulatorConstants.*;

public class ArmSubsystem extends SubsystemBase {
  private final CANSparkMax m_armMotor;
  private final RelativeEncoder m_armEncoder;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    m_armMotor = new CANSparkMax(kArmMotorPort, MotorType.kBrushed);
    m_armEncoder = m_armMotor.getEncoder();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setArmMotorSpeed(double speed) {
    m_armMotor.set(speed);
  }

  public double getArmPosition() {
    return m_armEncoder.getPosition();
  }

  public double getArmMotorCurrent() {
    SmartDashboard.putNumber("arm current", m_armMotor.getOutputCurrent());
    return m_armMotor.getOutputCurrent();
  }
}
