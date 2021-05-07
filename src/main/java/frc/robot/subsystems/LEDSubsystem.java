// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {

  public enum ColorPattern {

    Patriotic(Color.kRed, Color.kBlue, Color.kWhite),
    Christmas(Color.kRed, Color.kGreen),
    Cougartech(Color.kOrange, Color.kBlack),
    Snake(Color.kWhite, Color.kGreen, Color.kRed, Color.kGreen, Color.kRed, Color.kGreen, Color.kRed);

    private Color[] pattern;

    ColorPattern(Color... pattern) {
      this.pattern = pattern;
    }

    public Color[] getPattern() {
      return pattern;
    }
  }

  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
