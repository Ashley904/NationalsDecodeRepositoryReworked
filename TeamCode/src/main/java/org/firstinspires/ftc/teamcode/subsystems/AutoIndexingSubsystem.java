package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.robotConstants;

public class AutoIndexingSubsystem extends SubsystemBase {
    private final DistanceSensor leftDistanceSensor;
    private final DistanceSensor rightDistanceSensor;
    private final SpindexerSubsystem spindexerSubsystem;

    private int currentSpindexerIndex = 0;
    private boolean ballPresentLastCheck = false;
    private long lastMoveTime = 0;
    private long ballFirstDetectedTime = 0;
    private boolean ballContinuouslyPresent = false;

    private static final long FORCE_INDEX_TIMEOUT_MS = 460;

    public AutoIndexingSubsystem(HardwareMap hardwareMap, SpindexerSubsystem spindexerSubsystem) {
        this.spindexerSubsystem = spindexerSubsystem;

        leftDistanceSensor = hardwareMap.get(DistanceSensor.class, robotConstants.DistanceSensorConstants.leftDistanceSensorName);
        rightDistanceSensor = hardwareMap.get(DistanceSensor.class, robotConstants.DistanceSensorConstants.rightDistanceSensorName);
    }

    @Override
    public void periodic() {
        if (robotConstants.IntakeConstants.currentIntakeState != robotConstants.IntakeConstants.IntakeState.Intaking) return;

        if (spindexerSubsystem.velocityNearlyZeroed()) {
            double currentDistance = (leftDistanceSensor.getDistance(DistanceUnit.CM) + rightDistanceSensor.getDistance(DistanceUnit.CM)) / 2.0;
            boolean ballPresent = currentDistance <= robotConstants.DistanceSensorConstants.slotOccupiedThreshold;
            long currentTime = System.currentTimeMillis();

            // Track how long a ball has been continuously present
            if (ballPresent) {
                if (!ballContinuouslyPresent) {
                    ballFirstDetectedTime = currentTime;
                    ballContinuouslyPresent = true;
                }
            } else {
                ballContinuouslyPresent = false;
            }

            // Normal rising-edge detection OR forced index on timeout
            boolean risingEdge = ballPresent && !ballPresentLastCheck;
            boolean forceIndex = ballContinuouslyPresent && (currentTime - ballFirstDetectedTime >= FORCE_INDEX_TIMEOUT_MS);

            if ((risingEdge || forceIndex) && (currentTime - lastMoveTime > robotConstants.DistanceSensorConstants.spindexerIndexDelay)) {
                currentSpindexerIndex = (currentSpindexerIndex + 1) % robotConstants.SpindexerConstants.spindexerIntakingPositions.length;
                spindexerSubsystem.setSpindexerPosition(robotConstants.SpindexerConstants.spindexerIntakingPositions[currentSpindexerIndex]);
                lastMoveTime = currentTime;

                // Reset the timeout tracker so it doesn't fire repeatedly
                ballContinuouslyPresent = false;
            }

            ballPresentLastCheck = ballPresent;
        }
    }
}