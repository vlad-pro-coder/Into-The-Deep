package org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.teamcode.IntoTheDeep.CameraPipelines.YellowSampleDetectionPipeline;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Pathing.PurePersuit;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Localizer;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.PriorityQueue;

@Config
public class AutoConstants {
    public static final double OVERHEAD_idle = 345,
                                EXTENSION_overbasket = 1,
                                EXTENSION_readytakesample = 0.3,
                                LIFT_midway = 300,
                                LIFT_high = 700,

                                EXTENSION_idle = 0;

    public static final double LIFT_preload = 670,
                                LIFT_firstsample = 820,
                                LIFT_secondsample = 820,
                                LIFT_thirdsample = 615;

    public static final double OVERHEAD_preload = 105,
            OVERHEAD_firstsample = 75,
            OVERHEAD_secondsample = 75,
            OVERHEAD_thirdsample = 115,

            OVERHEAD_fromsubtobasket = 105;



    //the 3 samples
    public static final SparkFunOTOS.Pose2D
            CHASSIS_sample1pos = new SparkFunOTOS.Pose2D(526,-265,Math.toRadians(67)),
            CHASSIS_sample2pos = new SparkFunOTOS.Pose2D(548,-303,Math.toRadians(81.5)),
            CHASSIS_sample3pos = new SparkFunOTOS.Pose2D(575,-284,Math.toRadians(100)),
            CHASSIS_turntosubmersibleforcontinuostaking = new SparkFunOTOS.Pose2D(507,-228,Math.toRadians(58));

    public static final double EXTENDO_sample1 = 880,
                                EXTENDO_sample2 = 735,
                                EXTENDO_sample3 = 730;

    public static final SparkFunOTOS.Pose2D TO_SUBMERSIBLE_intermediary_point = new SparkFunOTOS.Pose2D(0,0,0),
                                            TO_SUBMERSIBLE_Final_point = new SparkFunOTOS.Pose2D(0,0,0);


    public static final ArrayList<PurePersuit.Point> PUREPERSUIT_pathtosubmersible = new ArrayList<>(Arrays.asList(
            new PurePersuit.Point(445, -150),
            new PurePersuit.Point(-172,-1212),
            new PurePersuit.Point(-468, -1290)
            )),
            PUREPERSUIT_pathbacktobasket = new ArrayList<>(Arrays.asList(
                    new PurePersuit.Point(-468, -1290),
                    new PurePersuit.Point(-172,-1212),
                    new PurePersuit.Point(445, -150)
            ));


    public static final double HEADING_infrontofsubmersible = Math.toRadians(0),
                            HEADING_fromsubmersibletobasket = Math.toRadians(60);

    public static final double PUREPERSUIT_radius = 500;

    public static PriorityQueue<YellowSampleDetectionPipeline.SamplePoint> orderedSamples = new PriorityQueue<>();
    public static YellowSampleDetectionPipeline.SamplePoint besttxty = new YellowSampleDetectionPipeline.SamplePoint(0,0);

}
