package frc.robot.commands;

import java.util.function.Consumer;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;

public class SetReefPositionCommand extends Command {
    private final Supplier<Integer> reefStationSupplier;
    private final Supplier<Integer> reefPoleSupplier;
    private final Consumer<Integer> reefStationConsumer;
    private final Consumer<Integer> reefPoleConsumer;
    private final int newStationValue;
    private final int newPoleValue;

    /**
     * 
     */
    public SetReefPositionCommand(Supplier<Integer> reefStationSupplier, Supplier<Integer> reefPoleSupplier,
            Consumer<Integer> reefStationConsumer, Consumer<Integer> reefPoleConsumer, int newStationValue,
            int newPoleValue) {
        this.reefStationSupplier = reefStationSupplier;
        this.reefPoleSupplier = reefPoleSupplier;
        this.reefStationConsumer = reefStationConsumer;
        this.reefPoleConsumer = reefPoleConsumer;
        this.newStationValue = newStationValue;
        this.newPoleValue = newPoleValue;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);

        Logger.recordOutput("Commands/Active Command", "");
    }

    @Override
    public void execute() {
    }

    @Override
    public void initialize() {
        int stationValue = this.reefStationSupplier.get().intValue() + this.newStationValue;
        if (stationValue > 6)
            stationValue = 6;
        if (stationValue < 1)
            stationValue = 1;

        int poleValue = this.reefPoleSupplier.get().intValue() + this.newPoleValue;
        if (poleValue > 2)
            poleValue = 2;
        if (poleValue < 1)
            poleValue = 1;

        this.reefStationConsumer.accept(stationValue);
        this.reefPoleConsumer.accept(poleValue);

        Logger.recordOutput("Commands/Active Command", this.getName());
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
