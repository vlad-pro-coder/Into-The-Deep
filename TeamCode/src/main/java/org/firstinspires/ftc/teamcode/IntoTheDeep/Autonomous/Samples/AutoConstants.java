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
    public static final double OVERHEAD_overbasket = 70,
                                OVERHEAD_overbasketbeforeslam = 100,
                                OVERHEAD_idle = 354,
                                EXTENSION_overbasket = 0.8,
                                EXTENSION_readytakesample = 0.35,
                                LIFT_highbasket = 800,
                                EXTENSION_idle = 0;


    //the 3 samples
    public static final SparkFunOTOS.Pose2D
            CHASSIS_sample1pos = new SparkFunOTOS.Pose2D(561,-272,Math.toRadians(63)),
            CHASSIS_sample2pos = new SparkFunOTOS.Pose2D(588,-275,Math.toRadians(77)),
            CHASSIS_sample3pos = new SparkFunOTOS.Pose2D(610,-315,Math.toRadians(94.5)),
            CHASSIS_turntosubmersibleforcontinuostaking = new SparkFunOTOS.Pose2D(507,-228,Math.toRadians(58));

    public static final double EXTENDO_sample1 = 870,
                                EXTENDO_sample2 = 765,
                                EXTENDO_sample3 = 710;

    public static final SparkFunOTOS.Pose2D TO_SUBMERSIBLE_intermediary_point = new SparkFunOTOS.Pose2D(0,0,0),
                                            TO_SUBMERSIBLE_Final_point = new SparkFunOTOS.Pose2D(0,0,0);


    public static final ArrayList<PurePersuit.Point> PUREPERSUIT_pathtosubmersible = new ArrayList<>(Arrays.asList(
            new PurePersuit.Point(507,-228),
            new PurePersuit.Point(185,-923),
            new PurePersuit.Point(18, -1238),
            new PurePersuit.Point(-453, -1360)
            )),
            PUREPERSUIT_pathbacktobasket = new ArrayList<>(Arrays.asList(
                    new PurePersuit.Point(-453, -1360),
                    new PurePersuit.Point(18, -1238),
                    new PurePersuit.Point(185,-923),
                    new PurePersuit.Point(507, -228)
            ));


    public static final double HEADING_infrontofsubmersible = Math.toRadians(0),
                            HEADING_fromsubmersibletobasket = Math.toRadians(58);

    public static final double PUREPERSUIT_radius = 600;

    public static PriorityQueue<YellowSampleDetectionPipeline.SamplePoint> orderedSamples = new PriorityQueue<>();

}
