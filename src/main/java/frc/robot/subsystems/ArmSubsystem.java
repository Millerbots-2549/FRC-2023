// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ManipulatorConstants;

public class ArmSubsystem extends SubsystemBase {

  private final WPI_TalonSRX m_armMotor;
  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    m_armMotor = new WPI_TalonSRX(ManipulatorConstants.kArmMotorPort);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setArmMotorSpeed(double speed) {
    m_armMotor.set(speed);
  }

  public double getArmMotorCurrent() {
    SmartDashboard.putNumber("arm current", m_armMotor.getStatorCurrent());
    return m_armMotor.getStatorCurrent();
  }
}
