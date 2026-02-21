package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.robotConstants;

public class ShooterSubsystem extends SubsystemBase {
    private final HardwareMap hardwareMap;






    private final DcMotorEx leftFlyWheelMotor;
    private final DcMotorEx rightFlyWheelMotor;
    private final Servo pitcherServo;
    private final Servo indicatorLightServo;





    private double targetVelocity=0.0;
    private double lastError=0.0, integralSum=0.0;





    private final ElapsedTime timer;





    public ShooterSubsystem(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;

        indicatorLightServo = hardwareMap.get(Servo.class, robotConstants.ShooterConstants.indicatorLightServoName);
        pitcherServo = hardwareMap.get(Servo.class, robotConstants.ShooterConstants.pitcherServoName);

        leftFlyWheelMotor = hardwareMap.get(DcMotorEx.class, robotConstants.ShooterConstants.leftFlyWheelMotorName);
        leftFlyWheelMotor.setDirection(robotConstants.ShooterConstants.leftFlyWheelMotorInverted ? DcMotorEx.Direction.REVERSE : DcMotorEx.Direction.FORWARD);
        leftFlyWheelMotor.setZeroPowerBehavior(robotConstants.ShooterConstants.floatModeEnabled ? DcMotor.ZeroPowerBehavior.FLOAT : DcMotor.ZeroPowerBehavior.BRAKE);

        rightFlyWheelMotor = hardwareMap.get(DcMotorEx.class, robotConstants.ShooterConstants.rightFlyWheelMotorName);
        rightFlyWheelMotor.setDirection(robotConstants.ShooterConstants.rightFlyWheelMotorInverted ? DcMotorEx.Direction.REVERSE : DcMotorEx.Direction.FORWARD);
        rightFlyWheelMotor.setZeroPowerBehavior(robotConstants.ShooterConstants.floatModeEnabled ? DcMotor.ZeroPowerBehavior.FLOAT : DcMotor.ZeroPowerBehavior.BRAKE);

        timer = new ElapsedTime();
        timer.reset();
    }





    @Override
    public void periodic(){

        // Calling Functions
        UpdatePIDFLoop();
        UpdateShooterState();
        UpdateIndicatorLightServoColor();
    }





    private void UpdatePIDFLoop(){
        // Voltage Scaling
        double voltageRatio = getCurrentVoltage() / robotConstants.ShooterConstants.nominalVoltage;
        double maxAchievableVelocity = robotConstants.ShooterConstants.maxVelocityAtNominalVoltage / voltageRatio;
        double clampedTargetVelocity = Math.min(targetVelocity, maxAchievableVelocity);

        double error = clampedTargetVelocity - getCurrentVelocity();


        double PIDOutput = (robotConstants.ShooterConstants.flyWheelKp * error);
        double feedforward = 0.0;
        if (clampedTargetVelocity > 0) { feedforward = robotConstants.ShooterConstants.flyWheelKs + (clampedTargetVelocity * robotConstants.ShooterConstants.flyWheelKf); }

        double totalOutput = (PIDOutput + feedforward) * getVoltageCompensation();
        totalOutput = Math.max(-1.0, Math.min(1.0, totalOutput));

        leftFlyWheelMotor.setPower(totalOutput);
        rightFlyWheelMotor.setPower(totalOutput);

        lastError = error;
        timer.reset();
    }
    private void UpdateShooterState(){
        double currentVelocity = getCurrentVelocity();
        double error = Math.abs(targetVelocity - currentVelocity);

        if(error <= robotConstants.ShooterConstants.velocityTolerance) robotConstants.ShooterConstants.currentShooterState = robotConstants.ShooterConstants.ShooterState.TargetReached;
        else robotConstants.ShooterConstants.currentShooterState = robotConstants.ShooterConstants.ShooterState.Accelerating;
    }





    private void UpdateIndicatorLightServoColor(){
        if(robotConstants.ShooterConstants.currentShooterState == robotConstants.ShooterConstants.ShooterState.Accelerating) indicatorLightServo.setPosition(robotConstants.ShooterConstants.indicatorLightAcceleratingPosition);
        else if(robotConstants.ShooterConstants.currentShooterState == robotConstants.ShooterConstants.ShooterState.TargetReached) indicatorLightServo.setPosition(robotConstants.ShooterConstants.indicatorLightTargetReachedPositon);
    }





    //Helper Methods
    public double getLeftFlyWheelVelocity() { return leftFlyWheelMotor.getVelocity(); }
    public double getRightFlyWheelVelocity() { return rightFlyWheelMotor.getVelocity(); }
    public double getCurrentVelocity() { return (leftFlyWheelMotor.getVelocity() + rightFlyWheelMotor.getVelocity()) / 2.0; }

    private double getVoltageCompensation() { return robotConstants.ShooterConstants.nominalVoltage / getCurrentVoltage(); }
    private double getCurrentVoltage() {
        try {
            VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();
            return voltageSensor.getVoltage();
        } catch (Exception e) { return robotConstants.ShooterConstants.nominalVoltage; }
    }

    public void setTargetVelocity(double velocity) { targetVelocity = velocity; }
    public void setPitcherSrvoPosition(double servoPosition) { pitcherServo.setPosition(servoPosition); }
}