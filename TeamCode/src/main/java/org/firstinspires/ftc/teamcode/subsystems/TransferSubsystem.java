package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.robotConstants;

public class TransferSubsystem extends SubsystemBase {
    SpindexerSubsystem spindexerSubsystem;






    private final Servo transferServo;





    public boolean transferingSequenceCompleted=false;





    public TransferSubsystem(HardwareMap hardwareMap, SpindexerSubsystem spindexerSubsystem){
        this.spindexerSubsystem = spindexerSubsystem;
        transferServo = hardwareMap.get(Servo.class, robotConstants.TransferConstants.transferServoName);
    }





    public void TransferLegAction(){
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new InstantCommand(() -> transferServo.setPosition(robotConstants.TransferConstants.transferPosition)),
                        new WaitCommand(robotConstants.TransferConstants.transferServoRiseTime),
                        new InstantCommand(() -> transferServo.setPosition(robotConstants.TransferConstants.transferHomePosition))
                )
        );
    }





    public void StartTransferAction(){
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        // Shoot 1st Position
                        new InstantCommand(() -> transferingSequenceCompleted=false),
                        new InstantCommand(() -> spindexerSubsystem.setSpindexerPosition(robotConstants.SpindexerConstants.spindexerTransferingPositions[0])),
                        new WaitCommand(100),
                        new WaitUntilCommand(() -> spindexerSubsystem.velocityZeroed()),

                        new InstantCommand(this:: TransferLegAction),
                        new WaitCommand(robotConstants.TransferConstants.transferHomingTime),

                        // Shoot 2nd Position
                        new InstantCommand(() -> spindexerSubsystem.setSpindexerPosition(robotConstants.SpindexerConstants.spindexerTransferingPositions[1])),
                        new WaitCommand(100),
                        new WaitUntilCommand(() -> spindexerSubsystem.velocityZeroed()),

                        new InstantCommand(this:: TransferLegAction),
                        new WaitCommand(robotConstants.TransferConstants.transferHomingTime),

                        // Shoot 3rd Position
                        new InstantCommand(() -> spindexerSubsystem.setSpindexerPosition(robotConstants.SpindexerConstants.spindexerTransferingPositions[2])),
                        new WaitCommand(65),
                        new WaitUntilCommand(() -> spindexerSubsystem.velocityZeroed()),

                        new InstantCommand(this:: TransferLegAction),
                        new WaitCommand(robotConstants.TransferConstants.transferHomingTime),
                        new WaitCommand(65),

                        new InstantCommand(() -> spindexerSubsystem.setSpindexerPosition(robotConstants.SpindexerConstants.spindexerIntakingPositions[0])),
                        new InstantCommand(() -> transferingSequenceCompleted=true)
                )
        );
    }





    public void HomeTransfer() {
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new InstantCommand(() -> transferServo.setPosition(robotConstants.TransferConstants.transferHomePosition))
                )
        );
    }
}