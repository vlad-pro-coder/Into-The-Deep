package org.firstinspires.ftc.teamcode.IntoTheDeep.TeleOps.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.IntoTheDeep.ActionsCommandLineImplementation.Scheduler;
import org.firstinspires.ftc.teamcode.IntoTheDeep.ActionsCommandLineImplementation.Task;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Extendo;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Intake;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Lift;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Outtake;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.RobotInitializers;

@TeleOp
@Config
public class TestScheduler extends LinearOpMode {


    public static double cnt = 0;
    @Override
    public void runOpMode() throws InterruptedException {

        Scheduler line = new Scheduler();
        RobotInitializers.InitializeFull(hardwareMap);

        line.addTask(new Task() {
            @Override
            protected void Actions() {
                Lift.state = Lift.LIFTSTATES.FREEWILL;
                Lift.setLiftPos(600);
                cnt+=1;
            }

            @Override
            protected boolean Conditions() {
                return Lift.IsLiftDone(40);
            }
        }).addTask(new Task() {
            @Override
            protected void Actions() {
                Outtake.OverHead_BASKETMOVINGSAFEPOS();
                cnt+=1;
            }

            @Override
            protected boolean Conditions() {
                return Outtake.OverHeadDoneness();
            }
        });
        RobotInitializers.InitializeForOperation();

        while(opModeInInit()){
            Lift.update();
            Outtake.update();
        }

        waitForStart();

        while (opModeIsActive()){

            RobotInitializers.Dashtelemetry.addData("currentTask",cnt);

            Lift.update();
            Outtake.update();
            RobotInitializers.Dashtelemetry.update();

            line.update();
        }

    }
}
