package org.firstinspires.ftc.teamcode.IntoTheDeep.Climb;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.IntoTheDeep.ActionsCommandLineImplementation.Scheduler;

public class ClimbConstants {
    public static boolean ClimbUnderWay = false;
    public static double BAR1 = 350, BAR2 = 700;
    public static double pitch = 0,climbArmIntertia = 45;
    public static ElapsedTime time = new ElapsedTime();
    public static Scheduler tasks = new Scheduler();
}
