package org.firstinspires.ftc.teamcode.opmodes.autonomous.redSide;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.opmodes.autonomous.blueSide.BlueAllianceCloseZoneAutonomous;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.AutoIndexingSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;
import org.firstinspires.ftc.teamcode.util.autonomousConstants;
import org.firstinspires.ftc.teamcode.util.robotConstants;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

@Autonomous(name="Red 9 Ball Close Zone Autonomous", group="Red Side")
public class RedAllianceCloseZoneAutonomous extends OpMode {
    private IntakeSubsystem intakeSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private TransferSubsystem transferSubsystem;
    private SpindexerSubsystem spindexerSubsystem;
    private AutoIndexingSubsystem autoIndexingSubsystem;










    Follower follower;






    //----------Path Chains----------//
    private PathChain scorePreloadPathChain;

    private PathChain collect2ndSetPathChain;
    private PathChain score2ndSetPathChain;

    private PathChain collect3rdSetPathChain;
    private PathChain score3rdSetPathChain;
    //---------- ---------//





    private static final List<CalibrationPoints> calibrationPoints = new ArrayList<>();
    double dynamicTargetFlyWheelVelocity = 0.0, dynamicPitcherServoPosition = 0.0;







    @Override
    public void init(){
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        shooterSubsystem = new ShooterSubsystem(hardwareMap);
        spindexerSubsystem = new SpindexerSubsystem(hardwareMap);
        transferSubsystem = new TransferSubsystem(hardwareMap, spindexerSubsystem);
        autoIndexingSubsystem = new AutoIndexingSubsystem(hardwareMap, spindexerSubsystem);





        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(autonomousConstants.RedSideCloseZoneConstants.startingPose);





        transferSubsystem.HomeTransfer();
        spindexerSubsystem.AutonomousPeriodInitializeSpindexer();






        initializePaths();
        AutonomousPathing();
        initializeCalibrationPoints();
    }

    @Override
    public void start(){
        AutonomousPathing();
    }

    @Override
    public void loop(){
        follower.update();





        BackGroundOperations();
        TelemetryUpdating();






        CommandScheduler.getInstance().run();
    }

    private void AutonomousPathing(){
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new InstantCommand(() -> follower.followPath(scorePreloadPathChain, 1, true)),
                        new WaitUntilCommand(() -> !follower.isBusy()),
                        new WaitCommand(500),
                        new InstantCommand(() -> transferSubsystem.StartTransferAction()),






                        new WaitUntilCommand(() -> transferSubsystem.transferingSequenceCompleted),
                        new WaitCommand(500),
                        new InstantCommand(() -> follower.followPath(collect2ndSetPathChain, 0.5, true)).alongWith(
                                new InstantCommand(() -> robotConstants.IntakeConstants.currentIntakeState = robotConstants.IntakeConstants.IntakeState.Intaking)
                        ),
                        new WaitUntilCommand(() -> !follower.isBusy()),
                        new WaitCommand(650),
                        new InstantCommand(() -> robotConstants.IntakeConstants.currentIntakeState = robotConstants.IntakeConstants.IntakeState.Off),
                        new InstantCommand(() -> follower.followPath(score2ndSetPathChain, 1.0, false)),
                        new WaitUntilCommand(() -> !follower.isBusy()),
                        new WaitCommand(450),
                        new InstantCommand(() -> transferSubsystem.StartTransferAction()),






                        new WaitUntilCommand(() -> transferSubsystem.transferingSequenceCompleted),
                        new InstantCommand(() -> follower.followPath(collect3rdSetPathChain, 0.5, false)).alongWith(
                                new InstantCommand(() -> robotConstants.IntakeConstants.currentIntakeState = robotConstants.IntakeConstants.IntakeState.Intaking)
                        ),
                        new WaitUntilCommand(() -> !follower.isBusy()),
                        new WaitCommand(500),
                        new InstantCommand(() -> follower.followPath(score3rdSetPathChain, 1.0, false)).alongWith(
                                new InstantCommand(() -> robotConstants.IntakeConstants.currentIntakeState = robotConstants.IntakeConstants.IntakeState.Off)
                        ),
                        new WaitUntilCommand(() -> !follower.isBusy()),
                        new WaitCommand(450),
                        new InstantCommand(() -> transferSubsystem.StartTransferAction()),






                        new WaitUntilCommand(() -> transferSubsystem.transferingSequenceCompleted),
                        new WaitUntilCommand(() -> !follower.isBusy())
                )
        );

    }





    private void BackGroundOperations(){
        shooterSubsystem.setTargetVelocity(dynamicTargetFlyWheelVelocity);
        shooterSubsystem.setPitcherSrvoPosition(dynamicPitcherServoPosition);






        intakeSubsystem.periodic();
        shooterSubsystem.periodic();
        transferSubsystem.periodic();
        spindexerSubsystem.periodic();
        autoIndexingSubsystem.periodic();






        CalculateShooterParameters();
    }





    private void TelemetryUpdating(){
        telemetry.addData("Current X Position: ", follower.getPose().getX());
        telemetry.addData("Current Y Position: ", follower.getPose().getY());
        telemetry.addData("Distance To Goal: ", follower.getPose().distanceFrom(robotConstants.FieldConstants.blueGoalFieldPose));
        telemetry.addData("Current Heading: ", Math.toDegrees(follower.getPose().getHeading()));

        telemetry.update();
    }





    private void initializePaths(){
        //--------Score Preload--------//
        scorePreloadPathChain = follower.pathBuilder()
                .addPath(new BezierLine(autonomousConstants.RedSideCloseZoneConstants.startingPose, autonomousConstants.RedSideCloseZoneConstants.scorePreloadPose))
                .setLinearHeadingInterpolation(autonomousConstants.RedSideCloseZoneConstants.startingPose.getHeading(), autonomousConstants.RedSideCloseZoneConstants.scorePreloadPose.getHeading())
                .build();





        //--------Collect + Score 2nd Set--------//
        collect2ndSetPathChain = follower.pathBuilder()
                .addPath(new BezierLine(
                        autonomousConstants.RedSideCloseZoneConstants.scorePreloadPose,
                        autonomousConstants.RedSideCloseZoneConstants.collect2ndSetPose1))
                .setLinearHeadingInterpolation(
                        autonomousConstants.RedSideCloseZoneConstants.scorePreloadPose.getHeading(),
                        autonomousConstants.RedSideCloseZoneConstants.collect2ndSetPose1.getHeading())
                .addPath(new BezierLine(
                        autonomousConstants.RedSideCloseZoneConstants.collect2ndSetPose1,
                        autonomousConstants.RedSideCloseZoneConstants.collect2ndSetPose2))
                .setLinearHeadingInterpolation(
                        autonomousConstants.RedSideCloseZoneConstants.collect2ndSetPose1.getHeading(),
                        autonomousConstants.RedSideCloseZoneConstants.collect2ndSetPose2.getHeading())
                .build();

        score2ndSetPathChain = follower.pathBuilder()
                .addPath(new BezierLine(autonomousConstants.RedSideCloseZoneConstants.collect2ndSetPose2, autonomousConstants.RedSideCloseZoneConstants.score2ndSetPose))
                .setLinearHeadingInterpolation(autonomousConstants.RedSideCloseZoneConstants.collect2ndSetPose2.getHeading(), autonomousConstants.RedSideCloseZoneConstants.score2ndSetPose.getHeading())
                .build();





        //--------Collect + Score 2nd Set--------//
        collect3rdSetPathChain = follower.pathBuilder()
                .addPath(new BezierLine(
                        autonomousConstants.RedSideCloseZoneConstants.score2ndSetPose,
                        autonomousConstants.RedSideCloseZoneConstants.collect3rdSetPose1))
                .setLinearHeadingInterpolation(
                        autonomousConstants.RedSideCloseZoneConstants.score2ndSetPose.getHeading(),
                        autonomousConstants.RedSideCloseZoneConstants.collect3rdSetPose1.getHeading())
                .addPath(new BezierLine(
                        autonomousConstants.RedSideCloseZoneConstants.collect3rdSetPose1,
                        autonomousConstants.RedSideCloseZoneConstants.collect3rdSetPose2))
                .setLinearHeadingInterpolation(
                        autonomousConstants.RedSideCloseZoneConstants.collect3rdSetPose1.getHeading(),
                        autonomousConstants.RedSideCloseZoneConstants.collect3rdSetPose2.getHeading())
                .build();

        score3rdSetPathChain = follower.pathBuilder()
                .addPath(new BezierLine(autonomousConstants.RedSideCloseZoneConstants.collect3rdSetPose2, autonomousConstants.RedSideCloseZoneConstants.score3rdSetPose))
                .setLinearHeadingInterpolation(autonomousConstants.RedSideCloseZoneConstants.collect3rdSetPose2.getHeading(), autonomousConstants.RedSideCloseZoneConstants.score3rdSetPose.getHeading())
                .build();

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
        double distance = follower.getPose().distanceFrom(robotConstants.FieldConstants.redGoalFieldPose);

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
                double t = (distance - p1.distanceToGoal) / (p2.distanceToGoal - p1.distanceToGoal);

                dynamicTargetFlyWheelVelocity = p1.flyWheelVelocity + t * (p2.flyWheelVelocity - p1.flyWheelVelocity);
                dynamicPitcherServoPosition   = p1.servoPosition + t * (p2.servoPosition - p1.servoPosition);

                dynamicTargetFlyWheelVelocity = Math.max(0, Math.min(1350.0, dynamicTargetFlyWheelVelocity));
                dynamicPitcherServoPosition   = Math.max(0, Math.min(0.65, dynamicPitcherServoPosition));
                return;
            }
        }
    }
}
