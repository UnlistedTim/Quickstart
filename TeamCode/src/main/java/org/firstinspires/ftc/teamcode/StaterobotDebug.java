package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Arrays;


//red right = top bb
//non red right = bot bb


//servo red dot middle 4
//servo red tape 2
//servo white tape0

//motor red dot 0
//motor two red dot 1
//motor one per side 2

//motor encoder full red0

//motor ecoder no red 1


@TeleOp
@Config
public class StaterobotDebug extends LinearOpMode {


    //blocker Block 0.25, blocker not block 0.65





    public DcMotorEx turretSpin;

    Pose2D pose;

    public GoBildaPinpointDriver Pinpoint;


    public static int IntakeVel = 0;
    int openInARow = 1;


    boolean limeValid = false;

    int id = 1;


    double Tx = 0;

    double Ty = 0;

    double rawIntakeCurrent;
    double filteredIntakeCurrent;


    public static double hoodPos = 0.0;// 0 vertical angle 1.0 horizontal angle

    public static double blockerPos = 0.37; // 0.46 open, 0.37 close

    public static double tripodPos = 1.0;   //1.0 collapse 0.35 extend

    public boolean prevBBState = true;

    public static double flyBotPower = 0;
    public static double flyTopPower = 0;

    public final int max_vel = 1800;

    public static int flyVel = 0;

    public int ball_count = 0;

    public boolean debounce = false;

    public boolean debouncearr[] = {false,false,false};

    int i = 0;


    private DcMotorEx Intake, flyBot, flyTop;

    MedianFilter intakeCurrentFilter = new MedianFilter(10);




    FtcDashboard dashboard = FtcDashboard.getInstance();


    Telemetry dashboardTelemetry = dashboard.getTelemetry();



    private Servo Hood, Blocker, Tripod;

    private DigitalChannel topBB;

    private DigitalChannel botBB;



//    private Limelight3A Limelight;



    public static double flyp = 0.002, flyi = 0, flyd = 0, flyf = 0.0005;

    PIDController flyPID = new PIDController(flyp, flyi, flyd);



    public class MedianFilter {
        private final double[] window;
        private int index = 0;
        private boolean filled = false;

        public MedianFilter(int size) {
            window = new double[size];
        }

        public double update(double value) {
            window[index] = value;
            index = (index + 1) % window.length;

            if (index == 0) {
                filled = true;
            }

            double[] temp = filled
                    ? window.clone()
                    : Arrays.copyOf(window, index);

            Arrays.sort(temp);

            return temp[temp.length / 2];
        }
    }





    @Override

    public void runOpMode() {
        Intake = hardwareMap.get(DcMotorEx.class, "Intake");

        Hood = hardwareMap.get(Servo.class, "Hood");
        Blocker = hardwareMap.get(Servo.class, "Blocker");
//        Flicker = hardwareMap.get(Servo.class, "Flicker");
        Tripod = hardwareMap.get(Servo.class, "Tripod");
        botBB = hardwareMap.get(DigitalChannel.class, "botBB");
        topBB = hardwareMap.get(DigitalChannel.class, "topBB");
        flyBot = hardwareMap.get(DcMotorEx.class, "flyBot");
        flyTop = hardwareMap.get(DcMotorEx.class, "flyTop");

//        Limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        turretSpin = hardwareMap.get(DcMotorEx.class, "turretSpin");

        Pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "Pinpoint");

//        configurePinpoint();



        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());//todo


        flyBot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        flyBot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flyTop.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flyBot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flyTop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        flyBot.setDirection(DcMotorSimple.Direction.REVERSE);
        flyTop.setDirection(DcMotorSimple.Direction.FORWARD);

        Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Intake.setVelocity(0);
        Intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        turretSpin.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        turretSpin.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretSpin.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        Hood.setDirection(Servo.Direction.REVERSE);

//        Limelight.pipelineSwitch(6);







        waitForStart();

//        Pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 88.83, 10.33, AngleUnit.RADIANS, 3*Math.PI/2));
//
//        Limelight.start();


        while (opModeIsActive()) { //Main While loop
            Hood.setPosition(hoodPos);

            Tripod.setPosition(tripodPos);

            Intake.setVelocity(IntakeVel);

            Blocker.setPosition(blockerPos);







            double turretPos = turretSpin.getCurrentPosition();
//
//            rawIntakeCurrent = Intake.getCurrent(CurrentUnit.MILLIAMPS);
//
//            filteredIntakeCurrent = intakeCurrentFilter.update(rawIntakeCurrent);




            flyPID(flyVel);

            telemetry.addData("Bottom BB state",botBB.getState());
            telemetry.addData("top BB state",topBB.getState());

            telemetry.update();





//            flyBot.setPower(flyBotPower);
//            flyTop.setPower(flyTopPower);
            //











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


        dashboardTelemetry.addData(" Current velocity", vel);
        dashboardTelemetry.addData("Target velocity",targ_vel);
        dashboardTelemetry.addData("Power",power);

        dashboardTelemetry.addData("Top CUrrent", flyTop.getCurrent(CurrentUnit.MILLIAMPS));
        dashboardTelemetry.addData("Bot CUrrent", flyBot.getCurrent(CurrentUnit.MILLIAMPS));

        dashboardTelemetry.addData("Intake vel",Intake.getVelocity());
        dashboardTelemetry.update();



        //  telemetry.addData("Velocity",vel);
    }

    public void configurePinpoint(){
        /*
         *  Set the odometry pod positions relative to the point that you want the position to be measured from.
         *
         *  The X pod offset refers to how far sideways from the tracking point the X (forward) odometry pod is.
         *  Left of the center is a positive number, right of center is a negative number.
         *
         *  The Y pod offset refers to how far forwards from the tracking point the Y (strafe) odometry pod is.
         *  Forward of center is a positive number, backwards is a negative number.
         */
        Pinpoint.setOffsets(3.15, -5, DistanceUnit.INCH); //these are tuned for 3110-0002-0001 Product Insight #1

        /*
         * Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
         * the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
         * If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
         * number of ticks per unit of your odometry pod.  For example:
         *     pinpoint.setEncoderResolution(13.26291192, DistanceUnit.MM);
         */
        Pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        /*
         * Set the direction that each of the two odometry pods count. The X (forward) pod should
         * increase when you move the robot forward. And the Y (strafe) pod should increase when
         * you move the robot to the left.
         */
        Pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);

        /*
         * Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
         * The IMU will automatically calibrate when first powered on, but recalibrating before running
         * the robot is a good idea to ensure that the calibration is "good".
         * resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
         * This is recommended before you run your autonomous, as a bad initial calibration can cause
         * an incorrect starting value for x, y, and heading.
         */
        Pinpoint.resetPosAndIMU();
    }

    public double calcDist(double x0, double y0, double x1, double y1){
        return Math.sqrt (Math.pow(x1 - x0, 2) + Math.pow(y1 - y0, 2));

    }





}









