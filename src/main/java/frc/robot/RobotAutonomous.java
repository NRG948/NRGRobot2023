package frc.robot;

import java.util.EnumSet;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class RobotAutonomous {

    private static enum ChooseAutoDelay {
        NO_DELAY(0),
        DELAY_2_SECONDS(2),
        DELAY_5_SECONDS(5);

        private int delay;

        ChooseAutoDelay(int delay) {
            this.delay = delay;
        }

        public int getDelay() {
            return delay;
        }

        public String toString() {
            return String.format("%d seconds", delay);
        }
    }

    private static SendableChooser<ChooseAutoDelay> chooseAutoDelay = new SendableChooser<>();

    private static Command getSelectedDelayCommand() {
        ChooseAutoDelay delayChoice = chooseAutoDelay.getSelected();
        return new WaitCommand(delayChoice.getDelay());
    }

    public static void addShuffleboardLayout(ShuffleboardLayout layout) {
        for (ChooseAutoDelay delay : EnumSet.allOf(ChooseAutoDelay.class)) {
            chooseAutoDelay.addOption(delay.toString(), delay);
        }

        chooseAutoDelay.setDefaultOption(ChooseAutoDelay.NO_DELAY.toString(), ChooseAutoDelay.NO_DELAY);

        layout.add("Delay", chooseAutoDelay);
    }
}
