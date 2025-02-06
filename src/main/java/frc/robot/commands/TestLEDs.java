// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ILEDPreset;
import frc.robot.subsystems.LEDPreset;
import frc.robot.subsystems.LEDs;

public class TestLEDs extends Command {
    private LEDs leds;
    private ILEDPreset colour;

    public TestLEDs(LEDs leds, ILEDPreset colour) {
        this.leds = leds;
        this.colour = colour;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // leds.setLEDS(LEDPreset.Solid.kOrange);//Is automaticly overrun by execute,
        // best used for values needed to defalt to a certain number
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        leds.setLEDS(colour);// Sets to colour imputed
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        leds.setLEDS(LEDPreset.Solid.kBlack);// if interupted, set light to black/off
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
