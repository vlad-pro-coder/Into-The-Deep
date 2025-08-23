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
    public static final double OVERHEAD_idle = 348,
                                EXTENSION_overbasket = 1,
                                EXTENSION_readytakesample = 0.25,
                                LIFT_midway = 300,
                                LIFT_high = 700,

                                EXTENSION_idle = 0;

    public static final double LIFT_preload = 680,
                                LIFT_firstsample = 820,
                                LIFT_secondsample = 820,
                                LIFT_thirdsample = 615;

    public static final double OVERHEAD_preload = 95,
            OVERHEAD_firstsample = 80,
            OVERHEAD_secondsample = 80,
            OVERHEAD_thirdsample = 120,

            OVERHEAD_fromsubtobasket = 110;


    public static final double EXTENSION_secondsample = 0.9;



    //the 3 samples
    public static final SparkFunOTOS.Pose2D
            CHASSIS_sample1pos = new SparkFunOTOS.Pose2D(486,-305,Math.toRadians(68)),
            CHASSIS_sample2pos = new SparkFunOTOS.Pose2D(548,-303,Math.toRadians(82)),
            CHASSIS_sample3pos = new SparkFunOTOS.Pose2D(575,-284,Math.toRadians(102)),
            CHASSIS_turntosubmersibleforcontinuostaking = new SparkFunOTOS.Pose2D(527,-208,Math.toRadians(58));

    public static final double EXTENDO_sample1 = 880,
                                EXTENDO_sample2 = 735,
                                EXTENDO_sample3 = 730;


    public static final ArrayList<PurePersuit.Point> PUREPERSUIT_pathtosubmersible = new ArrayList<>(Arrays.asList(
            new PurePersuit.Point(570, -100),
            new PurePersuit.Point(20,-931),
            new PurePersuit.Point(-430, -1290)
            )),
            PUREPERSUIT_pathbacktobasket = new ArrayList<>(Arrays.asList(
                    new PurePersuit.Point(-430, -1290),
                    new PurePersuit.Point(20,-931),
                    new PurePersuit.Point(570, -100)
            ));


    public static final double HEADING_infrontofsubmersible = Math.toRadians(22),
                            HEADING_fromsubmersibletobasket = Math.toRadians(57);

    public static final double PUREPERSUIT_radius = 500;
    public static YellowSampleDetectionPipeline.SamplePoint besttxty = new YellowSampleDetectionPipeline.SamplePoint(0,0);

    public static final double  OVERHEAD_startingpos = 270;

}
