package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.util.InterpLUT;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

//test


@TeleOp(name="TeleopState", group="A")
@Config
public class TeleopState extends LinearOpMode {


    private DcMotorEx Intake, flyBot, flyTop, turretSpin, leftFront, rightFront, leftBack, rightBack;

    private Servo Hood, Blocker, Tripod, Flicker;


    public double blockClose = 0.18, blockOpen = 0.4;

    public double flickerUp = 0.65, flickerDown = 0.38;

    public double tripodIdle = 0.95, tripodPark = 0.27;


    // lift pos 0.07 lift angle 40

    double Tyaverage = 0;

    public static double intakePower = 0.8;
    // private PIDController controller;
    public final int max_vel = 1800;

    double[] stoptime = new double[]{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    double[] Tydata = new double[]{0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    double[] Tyempty = new double[]{0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    int tyorder = 0;
    int intake = 0, outtake = 1, spinstatus = 2, spinfix = 3, shootbreak = 4;
    int pattern_id = 21, last_ball_number;
    public double hoodFar = 0.62;

    public double hoodLastPos = 0.0;

    public static double hoodPos = 0.4;
    boolean[] flag = new boolean[]{false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false};


    boolean recevieinfo = false;
    int[] shoot_order = new int[]{4, 4, 4, 4};
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    double currentintakepower = 0, MAX_TURN_POWER = 0.3;
    boolean limeValid = false;
    boolean drive = true, present = false;
    int presentpurple = 0, shoot_count = 0;
    int id = 1;
    int target_id = 24;
    int green_index = 4;
    public static double flyp = 0.008, flyi = 0, flyd = 0, flyf = 0.00045;


    boolean shootingActive = false;
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime runtime = new ElapsedTime();
    double Tx = 100;
    double Tx_offset = 0;
    double Ty = 0.0;
    double angle_to_goal = 0.0;
    InterpLUT Flylut = new InterpLUT();
    InterpLUT Hoodlut = new InterpLUT();
    double InterpPower = 0.0;
    boolean red = true;


    public IMU imu;


    // --- PID Control ---
    public static double turretkP = 0.025, turretkI = 0.05, turretkD = 0.002;
    PIDController turretPID = new PIDController(turretkP, turretkI, turretkD);
    PIDController flyPID = new PIDController(flyp, flyi, flyd);

    // --- State Variables ---
    public static double fly_factor = 0.0;
    public static double turretPower = 0.0;
    public static double targetTurretPos = 0.0;
    public int turretPos = 0;
    public double flywheelspeed = 0;

    private Limelight3A limelight;


    public enum State {
        DEBUG,
        IDLE,
        INTAKE,
        SORT,
        OUTTAKE,
        MANUALOUTTAKE;
    }

    //  State state = State.DEBUG;
    State state = State.IDLE;

    // --- Timers ---
    private ElapsedTime deltaT = new ElapsedTime();
    private ElapsedTime outtakeTimer = new ElapsedTime();


    @SuppressLint("SuspiciousIndentation")
    @Override

    public void runOpMode() {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        initalize();

        waitForStart();
        afterstart();
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        } //Bulk reading for faster loop times


        while (opModeIsActive()) { //Main While loop


            switch (state) {
                case DEBUG:


                    break;
                case IDLE:

                    intakeStart();
                    state = State.INTAKE;

                    break;


                case INTAKE:

                    break;
                case OUTTAKE:


                    break;

                case MANUALOUTTAKE:


                    break;


            }



            mecanumRobotDrive(-gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);
//
//
//            if (gamepad1.psWasPressed())resetOTOS();
//            if(gamepad2.psWasPressed()) flag[manualshoot] = !flag[manualshoot];
//            if (gamepad1.touchpadWasPressed()){
//                if (fieldCentric) fieldCentric = false;
//                else fieldCentric = true;
//            }


        }
    }


    public void flyprepower(double power) {
        flyBot.setPower(power);
        flyTop.setPower(power);
    }


    public void afterstart() {
        
        Blocker.setPosition(blockClose);

        deltaT.reset();
        timer.reset();
        outtakeTimer.reset();

        runtime.reset();

    }

    public void intakeStart(){
        Intake.setPower(intakePower);
    }


    public void initalize() {
        Hw_init();
        flyPID.setPID(flyp, flyi, flyd);
        try {
            red = (boolean) blackboard.get("RED");

        } catch (NullPointerException e) {
            red = true;
            telemetry.addLine("Color transfer Error!");

        }
        try {
            pattern_id = (int) blackboard.get("ID");
        } catch (NullPointerException e) {
            pattern_id = 21;
            telemetry.addLine("Pattern ID transfer Error!");

        }

        if (red) telemetry.addLine("Red Alliance Selected");
        else telemetry.addLine("Blue Alliance Selected");
        telemetry.addLine("Blue Alliance Selected");
        telemetry.addData(" Patter Green ", pattern_id - 20);
        telemetry.addLine("*******************************************");
        configinfo();
        telemetry.update();

        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.cross) {
                red = false;
                recevieinfo = true;
                telemetry.addLine("Blue Selected");
            }
            if (gamepad1.circle) {
                red = true;
                recevieinfo = true;
                telemetry.addLine("Red  Selected");
            }
            if (gamepad2.triangleWasPressed()) {
                pattern_id = 21;
                recevieinfo = true;
                telemetry.addLine(" Green 1 selected");
            }
            if (gamepad2.circleWasPressed()) {
                pattern_id = 22;
                recevieinfo = true;
                telemetry.addLine("  Green 2  selected");
            }
            if (gamepad2.crossWasPressed()) {
                pattern_id = 23;
                recevieinfo = true;
                telemetry.addLine(" Green 3 selected");
            }
            if (recevieinfo) {
                configinfo();
                telemetry.update();
            }
            if (gamepad1.right_bumper) break;
        }
        if (red) {
            Tx_offset = 0;
            target_id = 24;
//            limelight.pipelineSwitch(6);
        } else {
            Tx_offset = 0;
            target_id = 20;
//            limelight.pipelineSwitch(7);
        }

//        limelight.start();//todo
        telemetry.clear();

        if (red) telemetry.addLine("Red Alliance Selected");
        else telemetry.addLine("Blue Alliance Selected");
        telemetry.addLine("Blue Alliance Selected");
        telemetry.addData(" Patter Green ", pattern_id - 20);
        telemetry.update();

    }

    void configinfo() {
        telemetry.addLine("Driver Cross select Blue side");
        telemetry.addLine("Driver Circle select  Red  side");
        telemetry.addLine("Driver Triangle select  Debug mode");
        telemetry.addLine("Gunner Triangle select Green1 1");
        telemetry.addLine("Gunner Circle select Green 2");
        telemetry.addLine("Gunner Cross select Green3");
        telemetry.addLine("Drive Right Bumper Confrim ");


    }

//
//    public void manualturn(double x) {
//        if (x > 0.4) {//clockwise
//            turetturndir = -1;
//            flag[dected]=false;
//         //   turretSpin.setPower(-0.3);
//        //flag[manualturn]=true;
//            turettarget=turrretclock+10;
//        }
//
//        if (x < -0.4) {//couterclockwise
//            turetturndir = 1;
//           // turretSpin.setPower(0.3);
//          //  flag[manualturn]=true;
//            turettarget=turretcounterclock-10;
//            flag[dected]=false;
//        }
//
//
//    }
//
//
//    public void manualturnadj(double x) {
//        turretPos=turretSpin.getCurrentPosition();
//        double turnp= -0.35*x;
//
//
//
//        if(turretPos>turretcounterclock-30&& turnp>0)  {turnp=-0.3;sleep(50);}
//        if(turretPos<(turrretclock+30)&& turnp<0)  {turnp=-0.3;sleep(50);}
//
//        turretSpin.setPower(turnp);
//
//    }


//    public void ATy()
//
//    {
//        if(flag[Tylock]||!limeValid) return;
//        if(Ty<-15 || Ty>11) return;
//
//
//        double sum=0,count=0;
//        Tyaverage=0;
//        for(int i=0;i<10;i++)
//
//
//        {   if(i<9)
//            Tydata[i]=Tydata[i+1];
//            else Tydata[i]=Ty;
//            if(Ty>-15 && Ty<11) count++;
//            sum=sum+Tydata[i];
//
//
//        }
//        Tyaverage=sum/count;
//
//
//
//    }

    public void mecanumFieldDrive(double y, double x, double rx){
//        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
//        double x = gamepad1.left_stick_x;
//        double rx = gamepad1.right_stick_x;

        double botHeading = 0;

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;
        leftFront.setPower(frontLeftPower);
        leftBack.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightBack.setPower(backRightPower);

    }

    public void mecanumRobotDrive(double y, double x, double rx){
//        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
//        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
//        double rx = gamepad1.right_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        leftFront.setPower(frontLeftPower);
        leftBack.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightBack.setPower(backRightPower);
    }


    public void flywheel() {


        if (Tyaverage < 11 && Tyaverage > -15) InterpPower = Flylut.get(Tyaverage);


        else InterpPower = 0.75;

        if (Tyaverage < 11 && Tyaverage > -10.5) {

            double hoodLutGet = Hoodlut.get(Tyaverage);

            if (Math.abs(hoodLutGet - hoodLastPos) > 0.01) {
                Hood.setPosition(hoodLutGet);
                hoodLastPos = hoodLutGet;
            }

        } else if (Tyaverage < -10.5) Hood.setPosition(hoodFar);

        InterpPower = Math.round(InterpPower / 0.001) * 0.001;

        flywheelspeed = flyBot.getVelocity();

        flyPID(InterpPower);

    }


    public void Hw_init() {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        Intake = hardwareMap.get(DcMotorEx.class, "Intake");
        flyBot = hardwareMap.get(DcMotorEx.class, "flyCon");
        flyTop = hardwareMap.get(DcMotorEx.class, "flyExp");
        turretSpin = hardwareMap.get(DcMotorEx.class, "turretSpin");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());//todo


        Hood = hardwareMap.get(Servo.class, "Hood");
        Blocker = hardwareMap.get(Servo.class, "Blocker");
        Flicker = hardwareMap.get(Servo.class, "Flicker");
        Tripod = hardwareMap.get(Servo.class, "Tripod");

//        limelight = hardwareMap.get(Limelight3A.class, "Limelight");

        imu = hardwareMap.get(IMU.class, "imu");
        // This needs to be changed to match the orientation on your robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));


        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flyBot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        flyBot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flyTop.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flyBot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flyTop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        flyBot.setDirection(DcMotorSimple.Direction.FORWARD);
        flyTop.setDirection(DcMotorSimple.Direction.FORWARD);




        turretSpin.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        turretSpin.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretSpin.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


// far hood pos 0.48 power 0.9

        Flylut.add(-15, 0.92);  //far 0.89

        Flylut.add(-14, 0.895);  //far 0.87
        Flylut.add(-13.8, 0.9);  //far 0.85

        Flylut.add(-13, 0.878);  //far 0.85
        Flylut.add(-12.8, 0.863);  //far   0.82
        Flylut.add(-11.5, 0.82);  //far   0.82

        Flylut.add(-10.5, 0.77); //+1 // Input camera Ty, Output flywheel power
        Flylut.add(-9.55, 0.744);   // - 9.55 0.78 (2.0 hood)
        Flylut.add(-9.00, 0.735);   // - 9.55 0.78 (2.0 hood)
        Flylut.add(-8.70, 0.73);   // - 9.55 0.78 (2.0 hood)
        Flylut.add(-6.55, 0.7);// -6.55 0.74
        Flylut.add(-0.59, 0.63); // - 0.59 0.7
        Flylut.add(3.65, 0.6); // 3.65 0.67

        Flylut.add(11, 0.58); // 10 0.67 ; 0.0 hood

        // NEAR HOOD ANGLES
        Hoodlut.add(-10.5, 0.59);  //close

//        Hoodlut.add(-9.5 , 0.5);  //close

        Hoodlut.add(-6.5, 0.48);  //close

        Hoodlut.add(-0.65, 0.25);  //close
        Hoodlut.add(4, 0.19);  //close
        Hoodlut.add(11, 0);  //close


        Flylut.createLUT();

        Hoodlut.createLUT();


    }


    public boolean stoptimers(double period, int i) {

        if (period == 0) {
            stoptime[i] = runtime.milliseconds();
            return false;
        }
        return runtime.milliseconds() - stoptime[i] > period;
    }


    public void flyPID(double pow) {

        double target_vel = pow * max_vel;

        // double vel =  flyTop.getVelocity();
//
//        dashboardTelemetry.addData("Current speed", vel);
//        dashboardTelemetry.addData("Target speed", pow * max_vel);


        double pid = flyPID.calculate(flywheelspeed, target_vel);
        double ff = flyf * target_vel;
        double power = pid + ff;
        flyBot.setPower(power);
        flyTop.setPower(power);
    }
}




















