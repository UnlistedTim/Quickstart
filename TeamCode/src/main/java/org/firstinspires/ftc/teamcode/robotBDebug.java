package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;


@TeleOp
@Config
public class robotBDebug extends LinearOpMode {


    //blocker Block 0.25, blocker not block 0.65







    public static double IntakePower = 0.0;

    boolean limeValid = false;

    int id = 1;


    double Tx = 0;

    double Ty = 0;


    public static double hoodPos = 0.15;// 0 vertical angle 0.85 horizontal angle

    public static double blockerPos = 0.25; // 0.4 open 0.18 close

    public static double tripodPos = 0.95;

    public static double flyBotPower = 0;
    public static double flyTopPower = 0;

    public final int max_vel = 1800;

    public static int flyVel = 0;


    private DcMotorEx Intake, flyBot, flyTop;




    FtcDashboard dashboard = FtcDashboard.getInstance();


    Telemetry dashboardTelemetry = dashboard.getTelemetry();



    private Servo Hood, Blocker, Tripod;

    private Limelight3A Limelight;



    public static double flyp = 0.002, flyi = 0, flyd = 0, flyf = 0.0005;

    PIDController flyPID = new PIDController(flyp, flyi, flyd);




    @Override

    public void runOpMode() {
        Intake = hardwareMap.get(DcMotorEx.class, "Intake");

        Hood = hardwareMap.get(Servo.class, "Hood");
        Blocker = hardwareMap.get(Servo.class, "Blocker");
//        Flicker = hardwareMap.get(Servo.class, "Flicker");
        Tripod = hardwareMap.get(Servo.class, "Tripod");


        flyBot = hardwareMap.get(DcMotorEx.class, "flyBot");
        flyTop = hardwareMap.get(DcMotorEx.class, "flyTop");

        Limelight = hardwareMap.get(Limelight3A.class, "Limelight");



        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());//todo


        flyBot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        flyBot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flyTop.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flyBot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flyTop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        flyBot.setDirection(DcMotorSimple.Direction.REVERSE);
        flyTop.setDirection(DcMotorSimple.Direction.FORWARD);




        Hood.setDirection(Servo.Direction.REVERSE);

        Limelight.pipelineSwitch(6);





        waitForStart();
        Limelight.start();


        while (opModeIsActive()) { //Main While loop

            flyPID(flyVel);

//            flyBot.setPower(flyBotPower);
//            flyTop.setPower(flyTopPower);
            //
            Intake.setPower(IntakePower);


            Hood.setPosition(hoodPos);

            Blocker.setPosition(blockerPos);

            Tripod.setPosition(tripodPos);

            LLResult result = Limelight.getLatestResult();
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                id = fiducial.getFiducialId(); // The ID number of the fiducial
            }

            limeValid = result.isValid();

            if (limeValid){

                Tx = result.getTx();
                Ty = result.getTy();

            }

            dashboardTelemetry.addData("Tx", Tx);
            dashboardTelemetry.addData("Ty", Ty);

            telemetry.addData("Ty,",Ty);
            telemetry.update();
            dashboardTelemetry.addData("Pipeline",Limelight.pipelineSwitch(6));



            dashboardTelemetry.update();







////




        }
    }

    public void flyPID(double targ_vel){



        flyPID.setPID(flyp, flyi, flyd);
        double vel =  flyBot.getVelocity();

        double pid = flyPID.calculate(vel,targ_vel);
        double ff = flyf *targ_vel;
//        if (Math.abs(vel - targ_vel) < 40) pid = 0;
        double power = pid + ff;
        flyBot.setPower(power);
        flyTop.setPower(power);


        dashboardTelemetry.addData("Exp velocity", vel);
        dashboardTelemetry.addData("Target velocity",targ_vel);
        dashboardTelemetry.addData("Power",power);



        //  telemetry.addData("Velocity",vel);
    }





}









