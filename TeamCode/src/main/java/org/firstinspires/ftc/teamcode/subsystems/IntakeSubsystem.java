package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.robotConstants;

public class IntakeSubsystem extends SubsystemBase {
    HardwareMap hardwareMap;





    private final DcMotorEx intakeMotor;






    public IntakeSubsystem (HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;

        intakeMotor = hardwareMap.get(DcMotorEx.class, robotConstants.IntakeConstants.intakeMotorName);
        intakeMotor.setDirection(robotConstants.IntakeConstants.intakeMotorInverted ? DcMotorEx.Direction.REVERSE : DcMotorEx.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(robotConstants.IntakeConstants.floatModeEnabled ? DcMotor.ZeroPowerBehavior.FLOAT : DcMotor.ZeroPowerBehavior.BRAKE);
    }





    @Override
    public void periodic(){
        // Calling Functions
        UpdateIntakeSpeed();
    }





    private void UpdateIntakeSpeed(){
        if(robotConstants.IntakeConstants.currentIntakeState == robotConstants.IntakeConstants.IntakeState.Off) intakeMotor.setPower(robotConstants.IntakeConstants.disabledSpeed);
        else if(robotConstants.IntakeConstants.currentIntakeState == robotConstants.IntakeConstants.IntakeState.Intaking) intakeMotor.setPower(robotConstants.IntakeConstants.intakingSpeed);
        else if(robotConstants.IntakeConstants.currentIntakeState == robotConstants.IntakeConstants.IntakeState.Reversing) intakeMotor.setPower(robotConstants.IntakeConstants.reversingSpeed);
    }
}