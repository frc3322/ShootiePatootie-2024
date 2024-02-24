package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.*;

public class FloopIntakeCommands {

    private final FloopIntake floopIntake;

    public FloopIntakeCommands(FloopIntake floopIntake) {
        this.floopIntake = floopIntake;
    }

   

    public Command forceIntoShooter() {
        return new SequentialCommandGroup(
            new InstantCommand(
                ()-> {
                    floopIntake.intakeYawIn();
                }
            )
        );
    }

   

}