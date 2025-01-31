// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDPreset;
import frc.robot.subsystems.LEDs;

public class TestLEDs extends Command {
  private LEDs leds;

  public TestLEDs() {
    leds = LEDs.getInstance();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //leds.setLEDS(LEDPreset.Solid.kOrange);//Is automaticly overrun by execute
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    leds.setLEDS(LEDPreset.Solid.kYellow);
    //leds.setLEDS(LEDPreset.Color1And2.kNoBlend);//Current Funtion
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    leds.setLEDS(LEDPreset.Solid.kBlack);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
