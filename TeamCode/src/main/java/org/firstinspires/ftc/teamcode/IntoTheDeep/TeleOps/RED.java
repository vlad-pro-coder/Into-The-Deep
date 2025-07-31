package org.firstinspires.ftc.teamcode.IntoTheDeep.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Lift;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.RobotInitializers;
import org.firstinspires.ftc.teamcode.IntoTheDeep.TEAM;
import org.firstinspires.ftc.teamcode.IntoTheDeep.TeleopsStarter;

@TeleOp(name = ".peppersRED \uD83D\uDFE5")
public class RED extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        TeleopsStarter opmode = new TeleopsStarter(hardwareMap, TEAM.RED);

        waitForStart();
        RobotInitializers.enable();
        while (opModeIsActive()){
            opmode.update();
            RobotInitializers.telemetry.addData("Lift elevator", Lift.getPosition());
            RobotInitializers.telemetry.update();
        }
    }
}