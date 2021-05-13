// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Toolkit.CT_LEDStrip;
import frc.robot.Toolkit.CT_LEDStrip.ColorPattern;
import frc.robot.Toolkit.CT_LEDStrip.GlowColor;
import frc.robot.Toolkit.CT_LEDStrip.Speed;

public class LEDSubsystem extends SubsystemBase {

  public CT_LEDStrip m_led;
  public final Color[] redPattern = {Color.kRed, Color.kWhite};
  public final Color[] yellowPattern = {Color.kYellow, Color.kWhite};
  public final Color[] greenPattern = {Color.kGreen, Color.kWhite};
  public final Color[] bluePattern = {Color.kDarkBlue, Color.kWhite};

  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() {
    m_led = new CT_LEDStrip(1);

    m_led.addNormalPattern("Off", Color.kBlack);
    m_led.addNormalPattern("Red", Color.kRed);
    m_led.addMovingPattern("Christmas", Speed.Fast, ColorPattern.Christmas.getPattern());
    m_led.addSnakePattern("Snake", Speed.Ludicrous, Color.kBlack, ColorPattern.SnakeDefault.getPattern());
    m_led.addRainbowPattern();
    m_led.addGlowPattern("Yellow Glow", GlowColor.Red);
    //m_led.addSnakePattern("Pacman", Speed.Ludicrous, Color.kBlack, ColorPattern.SnakePacman.getPattern());
    
  }

  @Override
  public void periodic() {

    SmartDashboard.putString("Current Pattern", m_led.getCurrentPatternString());

    if(!RobotContainer.getShooterSubsystem().isShooting())
      m_led.getCurrentPattern().run();

  }

  public CT_LEDStrip getLEDStrip() {
    return m_led;
  }

}
