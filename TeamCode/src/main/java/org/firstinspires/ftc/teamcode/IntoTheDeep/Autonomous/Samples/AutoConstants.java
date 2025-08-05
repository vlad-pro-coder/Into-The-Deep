package org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

@Config
public class AutoConstants {
    public static final double OVERHEAD_overbasket = 90,
                                OVERHEAD_overbasketbeforeslam = 120,
                                OVERHEAD_idle = 355,
                                EXTENSION_overbasket = 0.8,
                                EXTENSION_readytakesample = 0.35,
                                LIFT_highbasket = 800,
                                EXTENSION_idle = 0;


    //the 3 samples
    public static final SparkFunOTOS.Pose2D
            CHASSIS_sample1pos = new SparkFunOTOS.Pose2D(561,-272,Math.toRadians(63)),
            CHASSIS_sample2pos = new SparkFunOTOS.Pose2D(588,-275,Math.toRadians(77)),
            CHASSIS_sample3pos = new SparkFunOTOS.Pose2D(610,-315,Math.toRadians(94.5));

    public static final double EXTENDO_sample1 = 870,
                                EXTENDO_sample2 = 765,
                                EXTENDO_sample3 = 710;

    public static final SparkFunOTOS.Pose2D TO_SUBMERSIBLE_intermediary_point = new SparkFunOTOS.Pose2D(0,0,0),
                                            TO_SUBMERSIBLE_Final_point = new SparkFunOTOS.Pose2D(0,0,0);

}
