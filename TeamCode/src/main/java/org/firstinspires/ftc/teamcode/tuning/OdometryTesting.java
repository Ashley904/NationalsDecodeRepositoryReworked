package org.firstinspires.ftc.teamcode.tuning;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name="Odometry Testing", group="Testing")
public class OdometryTesting extends LinearOpMode {
    Follower follower;





    Pose startingPose = new Pose(72.0, 72.0);
    
    
    
    
    
    @Override
    public void runOpMode(){
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        follower.update();

        telemetry.addData("Status: ", "ready to start...");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()){
            if(isStopRequested()) return;

            telemetry.addData("Current X Position: ", follower.getPose().getX());
            telemetry.addData("Current Y Position: ", follower.getPose().getY());
            telemetry.addData("Curent Heading: ", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.update();

            follower.update();
        }
    }
}
