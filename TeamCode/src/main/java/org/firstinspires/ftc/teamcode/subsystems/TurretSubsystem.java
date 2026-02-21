package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.robotConstants;

public class TurretSubsystem extends SubsystemBase {
    private final PIDController turretPIDController;
    private final DcMotor turretMotor;
    private double targetAngle = 0.0;

    private static final double ticksPerRevolution = 537.7;
    private static final double gearRatio          = 172.0 / 59.0;
    private static final double ticksPerRadian     = (ticksPerRevolution / (2 * Math.PI)) * gearRatio;

    public TurretSubsystem(HardwareMap hardwareMap) {
        turretPIDController = new PIDController(
                robotConstants.TurretConstants.turretKp,
                0.0,
                robotConstants.TurretConstants.turretKd
        );

        turretMotor = hardwareMap.get(DcMotor.class, robotConstants.TurretConstants.turretMotorName);
        turretMotor.setDirection(robotConstants.TurretConstants.turretMotorInverted
                ? DcMotor.Direction.REVERSE
                : DcMotorSimple.Direction.FORWARD);
        turretMotor.setZeroPowerBehavior(robotConstants.TurretConstants.floatModeEnabled
                ? DcMotor.ZeroPowerBehavior.FLOAT
                : DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // FIX: Use RUN_WITHOUT_ENCODER so only YOUR PID controls the motor.
        // RUN_USING_ENCODER layers the SDK's internal velocity PID on top of
        // your position PID = two competing control loops = jitter.
        // You can still READ the encoder in this mode.
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void periodic() {
        double targetTicks = Math.toRadians(targetAngle) * ticksPerRadian;
        turretPIDController.setSetPoint(targetTicks);

        double adjustment = turretPIDController.calculate(turretMotor.getCurrentPosition());

        double clampedAdjustment = Math.max(
                -robotConstants.TurretConstants.turretMaxPower,
                Math.min(robotConstants.TurretConstants.turretMaxPower, adjustment)
        );

        turretMotor.setPower(clampedAdjustment);
    }

    public void setTargetAngle(double angle) {
        targetAngle = Math.max(
                robotConstants.TurretConstants.turretLeftMaxAngle,
                Math.min(robotConstants.TurretConstants.turretRightMaxAngle, angle)
        );
    }

    public int currentTurretPosition() {
        return turretMotor.getCurrentPosition();
    }
}