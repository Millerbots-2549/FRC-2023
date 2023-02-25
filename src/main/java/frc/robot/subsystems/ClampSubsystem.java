// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ManipulatorConstants.*;

public class ClampSubsystem extends SubsystemBase {

  private final Solenoid m_clampSolenoid;
  private final PWMSparkMax m_leftClampMotor;
  private final PWMSparkMax m_rightClampMotor;

  /** Creates a new ClampSubsystem. */
  public ClampSubsystem() {

    m_clampSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, kClampSolenoidPort);
    m_leftClampMotor = new PWMSparkMax(kLeftClampMotorPort);
    m_rightClampMotor = new PWMSparkMax(kRightClampMotorPort);

    m_rightClampMotor.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void toggleSolenoid() {
    m_clampSolenoid.set(!m_clampSolenoid.get());
  }

  public void setSolenoidState(boolean state) {
    if (m_clampSolenoid.get() != state){
      m_clampSolenoid.set(state);
    }
  }

  public void setClampMotorSpeeds(double left, double right) {
    m_leftClampMotor.set(left);
    m_rightClampMotor.set(right);
  }
}
