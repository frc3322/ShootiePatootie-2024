// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;


public class ComboCommands{

    Intake intake;
    Elevator elevator;
    Transfer transfer;
    Shooter shooter;

    public ComboCommands(Elevator elevator, Intake groundIntake, Transfer transfer, Shooter shooter){
    this.elevator = elevator;
    this.intake = groundIntake;
    this.transfer = transfer;
    this.shooter = shooter;
    }

    /**
     * @return Command group to flip, put elevator down, and start intake to first beam break
     */
    public ParallelCommandGroup startAmpIntakeCommand() {
        return new ParallelCommandGroup(
            elevator.goToBottomCommand(),
            intake.flipToGroundAndRunPayloadCommand(
                intake.intakeToBeamBreak(), 
                0, 
                0
                )
        );
    }

     /**
     * @return Command group to flip, put elevator down, and start intake to the shooter beam break
     */
    public ParallelCommandGroup startShooterIntakeCommand() {
        return new ParallelCommandGroup(
            elevator.goToBottomCommand(),
            intake.flipToGroundAndRunPayloadCommand(
                intake.startSpin(), 
                0, 
                0
            ),
            transfer.intakeToShooterCommand()
        );
    } 

     /**
     * @return Command group to flip, put elevator down, and start intake to middle
     */
    public ParallelCommandGroup startMiddleIntakeCommand() {
        return new ParallelCommandGroup(
            elevator.goToBottomCommand(),
            intake.flipToGroundAndRunPayloadCommand(
                intake.intakeToMiddle(), 
                0, 
                0
            )
        );
    }

     /**
     * @return Command group to stop rollers and flip to stow
     */
    public ParallelCommandGroup stopIntakeCommand() {
        return new ParallelCommandGroup(
            intake.flipToStowAndRunPayloadCommand(
                intake.stopSpin(), 
                0, 
                0
            )
        );
    }

     /**
     * @return Command group to move elevator to amp position, flip to amp position, and eject note
     */
    public ParallelCommandGroup ampCommands() {
        return new ParallelCommandGroup(
            elevator.goToAmpCommand(),
            intake.flipToGroundAndRunPayloadCommand(
                intake.ejectCommand(), 
                .5, 
                .5
            )
        );
    }

     /**
     * @return Command group to move elevator down, flip intake down, and transfer to shooter beam break
     */
    public SequentialCommandGroup noteTransferToShooter() {
        return new ParallelCommandGroup(
            elevator.goToBottomCommand(),
            intake.flipToGroundAndRunPayloadCommand(
                intake.startSpin().until(intake::intakeEmpty), 
                .5, 
                0
            ),
            transfer.intakeToShooterCommand()
        ).andThen(stopIntakeCommand());
    }

     /**
     * @return Command group to move elevator down, flip intake down, and transfer to intake beam break
     */
    public SequentialCommandGroup noteTransferToIntake() {
        return new ParallelCommandGroup(
            elevator.goToBottomCommand(),
            intake.flipToGroundAndRunPayloadCommand(
                intake.ejectCommand().until(intake::outerIntakeFull), 
                .5, 
                .0
            ),
            transfer.shooterToIntakeCommand(intake::outerIntakeFull)
       ).andThen(stopIntakeCommand());
    }
}