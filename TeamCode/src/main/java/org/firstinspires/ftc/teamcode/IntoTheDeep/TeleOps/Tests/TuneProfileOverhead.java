package org.firstinspires.ftc.teamcode.IntoTheDeep.TeleOps.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Outtake;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.RobotInitializers;

@TeleOp
@Config
public class TuneProfileOverhead extends LinearOpMode {
    public static double a=0,v=0,d=0;
    public static double pos=0,lastpos = 0;
    public static ElapsedTime time = new ElapsedTime();
    public static double freq = 30;
    @Override
    public void runOpMode() throws InterruptedException {

        RobotInitializers.InitializeFull(hardwareMap);
        Outtake.setOverHeadPos(0);
        waitForStart();

        while (opModeIsActive()){
            if(lastpos != pos) {
                Outtake.armProfile.startMotion(Outtake.armProfile.getPosition(),pos);
                lastpos = pos;
            }
            Outtake.armProfile.acceleration = a;
            Outtake.armProfile.deceleration = d;
            Outtake.armProfile.maxVelocity = v;

            if(time.seconds() > 1.0/freq){
                time.reset();
                Outtake.armProfile.update();
                Outtake.Overheads1.setAngle(Outtake.armProfile.getPosition());
                Outtake.Overheads2.setAngle(Outtake.armProfile.getPosition());
            }

            RobotInitializers.Dashtelemetry.addData("pos",time.seconds());
            RobotInitializers.Dashtelemetry.update();
        }

    }
}
