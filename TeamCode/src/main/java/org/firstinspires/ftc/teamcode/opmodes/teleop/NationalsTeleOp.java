package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static java.lang.Math.sin;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.opmodes.autonomous.blueSide.BlueAllianceCloseZoneAutonomous;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.AutoIndexingSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.util.robotConstants;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

@Config
@TeleOp(name="Hyperion Bots Nationals TeleOp", group="Competition")
public class NationalsTeleOp extends OpMode {

    private GoBildaPinpointDriver pinpoint;

    FtcDashboard ftcDashboard;

    private TransferSubsystem transferSubsystem;
    private TurretSubsystem turretSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private SpindexerSubsystem spindexerSubsystem;
    private AutoIndexingSubsystem autoIndexingSubsystem;
    private ShooterSubsystem shooterSubsystem;

    private PIDController headingLockPIDController;

    private Follower follower;

    private DcMotor front_left_motor, back_left_motor, front_right_motor, back_right_motor;

    private boolean lockHeading = true;
    double targetRobotHeading;

    private ButtonReader enableIntakeButtonReader;
    private ButtonReader reverseIntakeButtonReader;
    private ButtonReader transferArtifactsButtonReader;
    private ButtonReader toggleDriveModeButtonReader;
    private ButtonReader offsetTurrefLeftButtonReader;
    private ButtonReader offsetTurrefRightButtonReader;
    private ButtonReader disableTUrretButtonReader;

    private double currentXPosition = 0.0, currentYPosition = 0.0, currentRobotHeading = 0.0;
    private double turretAngle = 0.0;

    private static final List<CalibrationPoints> calibrationPoints = new ArrayList<>();
    double dynamicTargetFlyWheelVelocity = 0.0, dynamicPitcherServoPosition = 0.0;





    private boolean disableTurret=false;



    @Override
    public void init() {
        CommandScheduler.getInstance().reset();

        dynamicTargetFlyWheelVelocity = 0.0;
        dynamicPitcherServoPosition = 0.0;

        ftcDashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        spindexerSubsystem = new SpindexerSubsystem(hardwareMap);
        turretSubsystem = new TurretSubsystem(hardwareMap);
        transferSubsystem = new TransferSubsystem(hardwareMap, spindexerSubsystem);
        autoIndexingSubsystem = new AutoIndexingSubsystem(hardwareMap, spindexerSubsystem);
        shooterSubsystem = new ShooterSubsystem(hardwareMap);

        headingLockPIDController = new PIDController(robotConstants.RobotConstants.headingKp, 0.0, robotConstants.RobotConstants.headingKd);

        front_left_motor  = hardwareMap.get(DcMotor.class, robotConstants.RobotConstants.frontLeftMotorName);
        back_left_motor   = hardwareMap.get(DcMotor.class, robotConstants.RobotConstants.backLeftMotorName);
        front_right_motor = hardwareMap.get(DcMotor.class, robotConstants.RobotConstants.frontRightMotorName);
        back_right_motor  = hardwareMap.get(DcMotor.class, robotConstants.RobotConstants.backRightMotorName);

        front_left_motor.setDirection(robotConstants.RobotConstants.frontLeftMotorInverted   ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        back_left_motor.setDirection(robotConstants.RobotConstants.backLeftMotorInverted     ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        front_right_motor.setDirection(robotConstants.RobotConstants.frontRightMotorInverted ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        back_right_motor.setDirection(robotConstants.RobotConstants.backRightMotorInverted   ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);

        front_left_motor.setZeroPowerBehavior(robotConstants.RobotConstants.floatModeEnabled  ? DcMotor.ZeroPowerBehavior.FLOAT : DcMotor.ZeroPowerBehavior.BRAKE);
        back_left_motor.setZeroPowerBehavior(robotConstants.RobotConstants.floatModeEnabled   ? DcMotor.ZeroPowerBehavior.FLOAT : DcMotor.ZeroPowerBehavior.BRAKE);
        front_right_motor.setZeroPowerBehavior(robotConstants.RobotConstants.floatModeEnabled ? DcMotor.ZeroPowerBehavior.FLOAT : DcMotor.ZeroPowerBehavior.BRAKE);
        back_right_motor.setZeroPowerBehavior(robotConstants.RobotConstants.floatModeEnabled  ? DcMotor.ZeroPowerBehavior.FLOAT : DcMotor.ZeroPowerBehavior.BRAKE);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(robotConstants.FieldConstants.robotStartingPose);

        initializeGamePadControls();
        initializeCalibrationPoints();
    }



    @Override
    public void init_loop() {
        if (gamepad1.x) robotConstants.RobotConstants.selectedAlliance = robotConstants.RobotConstants.Alliance.BlueAlliance;
        if (gamepad1.b) robotConstants.RobotConstants.selectedAlliance = robotConstants.RobotConstants.Alliance.RedAlliance;

        telemetry.addData("Selected Alliance (X=Blue / B=Red): ", robotConstants.RobotConstants.selectedAlliance);
        telemetry.update();

        shooterSubsystem.setTargetVelocity(0);
        shooterSubsystem.setPitcherSrvoPosition(0.6);

        turretSubsystem.setTargetAngle(0);

        spindexerSubsystem.setSpindexerPosition(robotConstants.SpindexerConstants.spindexerIntakingPositions[0]);
        robotConstants.IntakeConstants.currentIntakeState = robotConstants.IntakeConstants.IntakeState.Off;
    }



    @Override
    public void loop() {
        OdometryTracking();
        CalculateShooterParameters();
        RobotDrive();
        GamePadControlsManager();
        TelemtryUpdating();
        BackgroundOperations();
    }



    private void RobotDrive() {
        double adjustedDrivingSpeed = robotConstants.RobotConstants.driveCubicTerm * Math.pow(gamepad1.right_trigger, 3) + robotConstants.RobotConstants.driveLinearTerm * gamepad1.right_trigger;
        adjustedDrivingSpeed = 1.0 - adjustedDrivingSpeed;
        adjustedDrivingSpeed = Math.max(adjustedDrivingSpeed, robotConstants.RobotConstants.minimumDriveTrainSpeed);

        double y  = -gamepad1.left_stick_y * adjustedDrivingSpeed;
        double x  =  gamepad1.left_stick_x * adjustedDrivingSpeed;
        double rx = 0.0;

        double currentHeading = follower.getPose().getHeading();

        if (Math.abs(gamepad1.right_stick_x) > 0.01) {
            rx = gamepad1.right_stick_x * adjustedDrivingSpeed;
            lockHeading = false;
        } else {
            if (!lockHeading) targetRobotHeading = currentRobotHeading;
            lockHeading = true;

            double headingError = targetRobotHeading - currentHeading;
            headingError = Math.IEEEremainder(headingError, 2 * Math.PI);

            if (Math.abs(headingError) >= Math.toRadians(2)) { rx = headingLockPIDController.calculate(headingError); }
        }

        double offset = robotConstants.RobotConstants.selectedAlliance == robotConstants.RobotConstants.Alliance.BlueAlliance
                ? 90
                : 45;
        double rotatedX = x * Math.cos(-currentHeading + offset) - y * sin(-currentHeading + offset);
        double rotatedY = x * sin(-currentHeading + offset)      + y * Math.cos(-currentHeading + offset);

        double frontLeftMotorPower  = (robotConstants.RobotConstants.selectedDriveMode == robotConstants.RobotConstants.DriveMode.RobotCentric ? (y + x + rx) : (rotatedY + rotatedX + rx));
        double backLeftMotorPower   = (robotConstants.RobotConstants.selectedDriveMode == robotConstants.RobotConstants.DriveMode.RobotCentric ? (y - x + rx) : (rotatedY - rotatedX + rx));
        double frontRightMotorPower = (robotConstants.RobotConstants.selectedDriveMode == robotConstants.RobotConstants.DriveMode.RobotCentric ? (y - x - rx) : (rotatedY - rotatedX - rx));
        double backRightMotorPower  = (robotConstants.RobotConstants.selectedDriveMode == robotConstants.RobotConstants.DriveMode.RobotCentric ? (y + x - rx) : (rotatedY + rotatedX - rx));

        double maxPower = Math.max(
                Math.max(Math.abs(frontLeftMotorPower), Math.abs(backLeftMotorPower)),
                Math.max(Math.abs(frontRightMotorPower), Math.abs(backRightMotorPower))
        );

        if (maxPower > 1.0) {
            frontLeftMotorPower  /= maxPower;
            backLeftMotorPower   /= maxPower;
            frontRightMotorPower /= maxPower;
            backRightMotorPower  /= maxPower;
        }

        front_left_motor.setPower(frontLeftMotorPower);
        back_left_motor.setPower(backLeftMotorPower);
        front_right_motor.setPower(frontRightMotorPower);
        back_right_motor.setPower(backRightMotorPower);
    }



    private Pose getActiveGoalPose() {
        return robotConstants.RobotConstants.selectedAlliance == robotConstants.RobotConstants.Alliance.BlueAlliance
                ? robotConstants.FieldConstants.blueGoalFieldPose
                : robotConstants.FieldConstants.redGoalFieldPose;
    }

    private double calculateTurretAngle() {
        Pose goalPose = getActiveGoalPose();
        return Math.atan2(goalPose.getY() - currentYPosition, goalPose.getX() - currentXPosition) - currentRobotHeading;
    }

    private void OdometryTracking() {
        currentXPosition    = follower.getPose().getX();
        currentYPosition    = follower.getPose().getY();
        currentRobotHeading = follower.getPose().getHeading();
    }



    private void BackgroundOperations() {
        follower.update();

        if (gamepad1.start) { ResetRobotPose(); }

        headingLockPIDController.setPID(robotConstants.RobotConstants.headingKp, 0.0, robotConstants.RobotConstants.headingKd);

        shooterSubsystem.setTargetVelocity(dynamicTargetFlyWheelVelocity);
        shooterSubsystem.setPitcherSrvoPosition(dynamicPitcherServoPosition);

        double angle = Math.toDegrees(-turretAngle) + robotConstants.TurretConstants.trackingOffsset;
        turretSubsystem.setTargetAngle(angle);

        turretSubsystem.periodic();
        intakeSubsystem.periodic();
        spindexerSubsystem.periodic();
        autoIndexingSubsystem.periodic();
        shooterSubsystem.periodic();

        CommandScheduler.getInstance().run();
    }



    private void GamePadControlsManager() {
        enableIntakeButtonReader.readValue();
        if (enableIntakeButtonReader.wasJustPressed() && robotConstants.IntakeConstants.currentIntakeState == robotConstants.IntakeConstants.IntakeState.Off) robotConstants.IntakeConstants.currentIntakeState = robotConstants.IntakeConstants.IntakeState.Intaking;
        else if (enableIntakeButtonReader.wasJustPressed() && robotConstants.IntakeConstants.currentIntakeState == robotConstants.IntakeConstants.IntakeState.Intaking) robotConstants.IntakeConstants.currentIntakeState = robotConstants.IntakeConstants.IntakeState.Off;

        reverseIntakeButtonReader.readValue();
        if (reverseIntakeButtonReader.wasJustPressed() && robotConstants.IntakeConstants.currentIntakeState == robotConstants.IntakeConstants.IntakeState.Off) robotConstants.IntakeConstants.currentIntakeState = robotConstants.IntakeConstants.IntakeState.Reversing;
        else if (reverseIntakeButtonReader.wasJustPressed() && robotConstants.IntakeConstants.currentIntakeState == robotConstants.IntakeConstants.IntakeState.Reversing) robotConstants.IntakeConstants.currentIntakeState = robotConstants.IntakeConstants.IntakeState.Off;

        transferArtifactsButtonReader.readValue();
        if (transferArtifactsButtonReader.wasJustPressed()) { transferSubsystem.StartTransferAction(); }

        toggleDriveModeButtonReader.readValue();
        if (toggleDriveModeButtonReader.wasJustPressed() && robotConstants.RobotConstants.selectedDriveMode == robotConstants.RobotConstants.DriveMode.RobotCentric) robotConstants.RobotConstants.selectedDriveMode = robotConstants.RobotConstants.DriveMode.FieldCentric;
        else if (toggleDriveModeButtonReader.wasJustPressed() && robotConstants.RobotConstants.selectedDriveMode == robotConstants.RobotConstants.DriveMode.FieldCentric) robotConstants.RobotConstants.selectedDriveMode = robotConstants.RobotConstants.DriveMode.RobotCentric;

        offsetTurrefLeftButtonReader.readValue();
        offsetTurrefRightButtonReader.readValue();
        if (offsetTurrefLeftButtonReader.wasJustPressed()) { robotConstants.TurretConstants.trackingOffsset -= 4; }
        else if (offsetTurrefRightButtonReader.wasJustPressed()) { robotConstants.TurretConstants.trackingOffsset += 4; }


        disableTUrretButtonReader.readValue();
        if(disableTUrretButtonReader.wasJustPressed() && !disableTurret) { turretAngle = 0.0; }
        else if(disableTUrretButtonReader.wasJustPressed() && disableTurret) { turretAngle = calculateTurretAngle(); }


    }

    private void initializeGamePadControls() {
        robotConstants.GamePadControls.gamepad1EX = new GamepadEx(gamepad1);

        enableIntakeButtonReader      = new ButtonReader(robotConstants.GamePadControls.gamepad1EX, robotConstants.GamePadControls.enableIntakeMapping);
        reverseIntakeButtonReader     = new ButtonReader(robotConstants.GamePadControls.gamepad1EX, robotConstants.GamePadControls.reverseIntakeMapping);
        transferArtifactsButtonReader = new ButtonReader(robotConstants.GamePadControls.gamepad1EX, robotConstants.GamePadControls.transferArtifactsMapping);
        toggleDriveModeButtonReader   = new ButtonReader(robotConstants.GamePadControls.gamepad1EX, robotConstants.GamePadControls.switchDriveModeMapping);
        disableTUrretButtonReader = new ButtonReader(robotConstants.GamePadControls.gamepad1EX, robotConstants.GamePadControls.disableTurretMapping);

        offsetTurrefLeftButtonReader  = new ButtonReader(robotConstants.GamePadControls.gamepad1EX, robotConstants.GamePadControls.offsetTurretLeftMapping);
        offsetTurrefRightButtonReader = new ButtonReader(robotConstants.GamePadControls.gamepad1EX, robotConstants.GamePadControls.offsetTurretRightMapping);
    }



    private void TelemtryUpdating() {
        telemetry.addData("Selected Alliance: ", robotConstants.RobotConstants.selectedAlliance);
        telemetry.addData("Distance To Goal: ", follower.getPose().distanceFrom(getActiveGoalPose()));
        telemetry.addData("Current X Position: ", currentXPosition);
        telemetry.addData("Current Y Position: ", currentYPosition);
        telemetry.addData("Current Heading: ", Math.toDegrees(currentRobotHeading));
        telemetry.addData("Turret Heading Angle: ", Math.toDegrees(turretAngle));
        telemetry.addData("Current Shooter Velocity: ", shooterSubsystem.getCurrentVelocity());
        telemetry.addData("Target Velocity: ", dynamicTargetFlyWheelVelocity);
        telemetry.addData("Target Pitcher Position: ", dynamicPitcherServoPosition);
        telemetry.update();
    }



    private static class CalibrationPoints {
        public double distanceToGoal;
        public double flyWheelVelocity;
        public double servoPosition;

        public CalibrationPoints(double distanceToGoal, double flyWheelVelocity, double servoPosition) {
            this.distanceToGoal   = distanceToGoal;
            this.flyWheelVelocity = flyWheelVelocity;
            this.servoPosition    = servoPosition;
        }
    }

    private void initializeCalibrationPoints() {
        calibrationPoints.clear();

        calibrationPoints.add(new CalibrationPoints(127.0, 1055.0, 0.45));
        calibrationPoints.add(new CalibrationPoints(112.0, 985.0, 0.46));
        calibrationPoints.add(new CalibrationPoints(90.0, 930.0, 0.45));
        calibrationPoints.add(new CalibrationPoints(70.0, 890, 0.48));
        calibrationPoints.add(new CalibrationPoints(50.0, 825, 0.5));

        calibrationPoints.sort(Comparator.comparingDouble(a -> a.distanceToGoal));
    }

    private void CalculateShooterParameters() {
        double distance = follower.getPose().distanceFrom(getActiveGoalPose());

        if (calibrationPoints.isEmpty()) {
            dynamicTargetFlyWheelVelocity = 0.0;
            dynamicPitcherServoPosition   = 0.0;
            return;
        }

        if (distance <= calibrationPoints.get(0).distanceToGoal) {
            dynamicTargetFlyWheelVelocity = calibrationPoints.get(0).flyWheelVelocity;
            dynamicPitcherServoPosition   = calibrationPoints.get(0).servoPosition;
            return;
        }

        if (distance >= calibrationPoints.get(calibrationPoints.size() - 1).distanceToGoal) {
            CalibrationPoints last = calibrationPoints.get(calibrationPoints.size() - 1);
            dynamicTargetFlyWheelVelocity = last.flyWheelVelocity;
            dynamicPitcherServoPosition   = last.servoPosition;
            return;
        }

        for (int i = 0; i < calibrationPoints.size() - 1; i++) {
            CalibrationPoints p1 = calibrationPoints.get(i);
            CalibrationPoints p2 = calibrationPoints.get(i + 1);

            if (distance >= p1.distanceToGoal && distance <= p2.distanceToGoal) {
                CalibrationPoints p0 = (i > 0) ? calibrationPoints.get(i - 1) : p1;
                CalibrationPoints p3 = (i < calibrationPoints.size() - 2) ? calibrationPoints.get(i + 2) : p2;

                double t  = (distance - p1.distanceToGoal) / (p2.distanceToGoal - p1.distanceToGoal);
                double t2 = t * t;
                double t3 = t2 * t;

                double m1_vel = (p2.flyWheelVelocity - p0.flyWheelVelocity) / (p2.distanceToGoal - p0.distanceToGoal);
                double m2_vel = (p3.flyWheelVelocity - p1.flyWheelVelocity) / (p3.distanceToGoal - p1.distanceToGoal);

                dynamicTargetFlyWheelVelocity =
                        (2*t3 - 3*t2 + 1) * p1.flyWheelVelocity +
                                (t3 - 2*t2 + t)   * (p2.distanceToGoal - p1.distanceToGoal) * m1_vel +
                                (-2*t3 + 3*t2)    * p2.flyWheelVelocity +
                                (t3 - t2)         * (p2.distanceToGoal - p1.distanceToGoal) * m2_vel;

                double m1_servo = (p2.servoPosition - p0.servoPosition) / (p2.distanceToGoal - p0.distanceToGoal);
                double m2_servo = (p3.servoPosition - p1.servoPosition) / (p3.distanceToGoal - p1.distanceToGoal);

                dynamicPitcherServoPosition =
                        (2*t3 - 3*t2 + 1) * p1.servoPosition +
                                (t3 - 2*t2 + t)   * (p2.distanceToGoal - p1.distanceToGoal) * m1_servo +
                                (-2*t3 + 3*t2)    * p2.servoPosition +
                                (t3 - t2)         * (p2.distanceToGoal - p1.distanceToGoal) * m2_servo;

                dynamicTargetFlyWheelVelocity = Math.max(0, Math.min(2200, dynamicTargetFlyWheelVelocity));
                dynamicPitcherServoPosition   = Math.max(0, Math.min(1.0,  dynamicPitcherServoPosition));
                return;
            }
        }
    }



    private void ResetRobotPose() {
        follower.setPose(robotConstants.FieldConstants.robotStartingPose);
        targetRobotHeading = 0.0;
    }
}