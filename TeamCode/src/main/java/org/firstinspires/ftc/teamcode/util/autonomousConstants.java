package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.geometry.Pose;

public class autonomousConstants {





    public static class BlueSideCloseZoneConstants{
        public static Pose startingPose = new Pose(129.0, 107.0, Math.toRadians(47.15));
        public static Pose scorePreloadPose = new Pose(88.0, 77.0, Math.toRadians(46.1));





        public static Pose collect2ndSetPose1 = new Pose(88.0, 78.0, Math.toRadians(90.0));
        public static Pose collect2ndSetPose2 = new Pose(88.0, 103.0, Math.toRadians(90.0));
        public static Pose score2ndSetPose = new Pose(88.0, 77.0, Math.toRadians(46.1));





        public static Pose collect3rdSetPose1 = new Pose(65.1, 80.0, Math.toRadians(90.0));
        public static Pose collect3rdSetPose2 = new Pose(65.1, 106.50, Math.toRadians(90.0));
        public static Pose score3rdSetPose = new Pose(88.0, 80.0, Math.toRadians(46.1));





        public static Pose leaveStartingZonePose = new Pose(70, 80, Math.toRadians(90.0));
    }










    public static class RedSideCloseZoneConstants{
        public static Pose startingPose = new Pose(118, 130.0, Math.toRadians(38.0));
        public static Pose scorePreloadPose = new Pose(90, 90.0, Math.toRadians(39));





        public static Pose collect2ndSetPose1 = new Pose(87.0, 95.0, Math.toRadians(-6.0));
        public static Pose collect2ndSetPose2 = new Pose(120.0, 95.0, Math.toRadians(-6.0));
        public static Pose score2ndSetPose = new Pose(90.0, 90.0, Math.toRadians(37.95));





        public static Pose collect3rdSetPose1 = new Pose(87.0, 71.2, Math.toRadians(-6.0));
        public static Pose collect3rdSetPose2 = new Pose(120.0, 71.2, Math.toRadians(-6.0));
        public static Pose score3rdSetPose = new Pose(90.0, 90.0, Math.toRadians(37.95));
    }
}
