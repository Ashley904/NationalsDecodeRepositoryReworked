package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.util.robotConstants;

@Config
@TeleOp(name="Shooter Tuning", group="Tuning")
public class shooterTuning extends LinearOpMode {
    ShooterSubsystem shooterSubsystem;
    Follower follower;

    public static double shooterVelocity=0.0;
    public static double pitcherServoTargetPosition=0.0;

    private double distanceToGoal=0.0;

    FtcDashboard ftcDashboard;

    private Pose startingPose = new Pose(72.0, 72.0);

    @Override
    public void runOpMode(){
        ftcDashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        shooterSubsystem = new ShooterSubsystem(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        follower.update();

        telemetry.addData("Status: ", "Ready to start...");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()){
            if(isStopRequested()) return;
            shooterSubsystem.setTargetVelocity(shooterVelocity);
            shooterSubsystem.setPitcherSrvoPosition(pitcherServoTargetPosition);
            shooterSubsystem.periodic();

            follower.update();
            distanceToGoal = follower.getPose().distanceFrom(robotConstants.FieldConstants.blueGoalFieldPose);

            telemetry.addData("X: ", follower.getPose().getX());
            telemetry.addData("Y: ", follower.getPose().getY());
            telemetry.addData("Current Velocity: ", shooterSubsystem.getCurrentVelocity());
            telemetry.addData("Target Velocity: ", shooterVelocity);
            telemetry.addData("Distance To Goal: ", distanceToGoal);
            telemetry.update();
        }
    }
}