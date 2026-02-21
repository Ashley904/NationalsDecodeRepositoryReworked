package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.util.robotConstants;

public class SpindexerSubsystem extends SubsystemBase {
    private final HardwareMap hardwareMap;






    private final Servo leftSpindexerServo;
    private final Servo rightSpindexerServo;
    private final DcMotorEx encoder;






    public SpindexerSubsystem(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;

        leftSpindexerServo = hardwareMap.get(Servo.class, robotConstants.SpindexerConstants.leftSpindexerServoName);
        rightSpindexerServo = hardwareMap.get(Servo.class, robotConstants.SpindexerConstants.rightSpindexerServoName);

        encoder = hardwareMap.get(DcMotorEx.class, robotConstants.SpindexerConstants.encoderName);
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }





    public void setSpindexerPosition(double targetSpindexerPosition){
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new InstantCommand(() -> leftSpindexerServo.setPosition(targetSpindexerPosition)).alongWith(
                                new InstantCommand(() -> rightSpindexerServo.setPosition(targetSpindexerPosition))
                        )
                )
        );
    }





    public void AutonomousPeriodInitializeSpindexer(){
        leftSpindexerServo.setPosition(robotConstants.SpindexerConstants.spindexerIntakingPositions[0]);
        rightSpindexerServo.setPosition(robotConstants.SpindexerConstants.spindexerIntakingPositions[0]);
    }





    public double getEncoderVelocity(){ return encoder.getVelocity(); }




    public boolean velocityZeroed() {
        double currentVelocity = getEncoderVelocity();
        return Math.abs(currentVelocity) <= robotConstants.SpindexerConstants.spindexerVelocityThreshold;
    }
    public boolean velocityNearlyZeroed() {
        double currentVelocity = getEncoderVelocity();
        return Math.abs(currentVelocity) <= robotConstants.DistanceSensorConstants.spindexerVelocityZeroedTolerance;
    }
}