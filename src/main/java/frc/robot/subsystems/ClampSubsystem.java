// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ManipulatorConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ClampSubsystem extends SubsystemBase {

  private final Solenoid m_clampSolenoid;
  private final CANSparkMax m_leftClampMotor;
  private final CANSparkMax m_rightClampMotor;
  private final RelativeEncoder m_leftEncoder;
  private final RelativeEncoder m_rightEncoder;

  /** Creates a new ClampSubsystem. */
  public ClampSubsystem() {
    m_clampSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, kClampSolenoidPort);
    m_leftClampMotor = new CANSparkMax(kLeftClampMotorPort, MotorType.kBrushless);
    m_rightClampMotor = new CANSparkMax(kRightClampMotorPort, MotorType.kBrushless);
    m_leftEncoder = m_leftClampMotor.getEncoder();
    m_rightEncoder = m_rightClampMotor.getEncoder();

    m_rightClampMotor.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("clamp motor speeds", getAverageMotorSpeeds());
  }

  public void toggleSolenoid() {
    m_clampSolenoid.set(!m_clampSolenoid.get());
  }

  public void setSolenoidState(boolean state) {
    if (m_clampSolenoid.get() != state){
      m_clampSolenoid.set(state);
    }
  }

  public boolean isClampInCubeMode() {
    return m_clampSolenoid.get();
  }

  public void setClampMotorSpeeds(double speed, int amps) {
    m_leftClampMotor.set(speed);
    m_leftClampMotor.setSmartCurrentLimit(amps);
    m_rightClampMotor.set(speed);
    m_rightClampMotor.set(amps);
  }

  public double getAverageMotorSpeeds() {
    return (m_leftEncoder.getVelocity() + m_rightEncoder.getVelocity()) / 2;
  }
}
