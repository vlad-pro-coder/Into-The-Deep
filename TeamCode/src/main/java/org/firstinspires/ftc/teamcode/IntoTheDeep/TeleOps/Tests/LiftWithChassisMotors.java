package org.firstinspires.ftc.teamcode.IntoTheDeep.TeleOps.Tests;

import static org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.PtoAndWheelie.UpdateChassisDrivenLift;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Lift;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.PtoAndWheelie;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.RobotInitializers;

@TeleOp
@Config
public class LiftWithChassisMotors extends LinearOpMode {

    public static double pos = 0;
    public static double lastpos = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotInitializers.InitializeFull(hardwareMap);

        Lift.state = Lift.LIFTSTATES.OFF;

        PtoAndWheelie.engagePTO();

        waitForStart();

        while(opModeIsActive()){

            RobotInitializers.clearCache();

            if(pos != lastpos) {
                PtoAndWheelie.setPosChassisDrivenLift(pos);
                lastpos = pos;
            }

            UpdateChassisDrivenLift();
        }

    }
}
