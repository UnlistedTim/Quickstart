package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.TeleopStateA;
import org.firstinspires.ftc.teamcode.bass;


@Autonomous(name = " StateAuto", group = "A")
public class StateAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    public bass rbga;
    private DcMotorEx Intake, flyBot, flyTop, turretSpin, leftFront, rightFront, leftBack, rightBack;

    private Servo Hood, Blocker, Tripod;

    private DigitalChannel beamBreaker;

    private Limelight3A Limelight;

    private GoBildaPinpointDriver Pinpoint;
    public boolean red=true,recevieinfo=false;


    //TeleopStateA.BooleanConfidenceChecker checker = new TeleopStateA.BooleanConfidenceChecker();


    private int pathState;
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(3, 3, Math.toRadians(-20)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose pickup1Pose = new Pose(7, 22.5, Math.toRadians(90)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2Pose = new Pose(3, 38, Math.toRadians(90)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickup3Pose = new Pose(49, 135, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private Path scorePreload;
    private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3;



    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:

                //flywheel spee up, ID read,turn table turn right,shoot;
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:

            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score Preload */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup1, true);
                    setPathState(2);
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup1, true);
                    setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup2, true);
                    setPathState(4);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup2, true);
                    setPathState(-1);
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup3, true);
                    setPathState(6);
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup3, true);
                    setPathState(7);
                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
                }
                break;
        }
    }

    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work


        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {
        Hw_init();
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        rbga=new bass();


        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
        telemetry.addLine("Driver Cross select Blue side");
        telemetry.addLine("Driver Circle select  Red  side");

         telemetry.addLine("Drive Right Bumper Confrim ");
        telemetry.update();

        while(gamepad1.right_bumper ) {
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
//            if (gamepad2.triangleWasPressed()) {
//                pattern_id = 21;
//                recevieinfo = true;
//                telemetry.addLine(" Green 1 selected");
//            }
//            if (gamepad2.circleWasPressed()) {
//                pattern_id = 22;
//                recevieinfo = true;
//                telemetry.addLine("  Green 2  selected");
//            }
//            if (gamepad2.crossWasPressed()) {
//                pattern_id = 23;
//                recevieinfo = true;
//                telemetry.addLine(" Green 3 selected");
//            }
            if (recevieinfo) {
              //  configinfo();
                telemetry.update();
            }

        }


        if (red) {

            Limelight.pipelineSwitch(6);
        } else {

            Limelight.pipelineSwitch(7);
        }
        Limelight.start();

    }

    /**
     * This method is called continuously after Init while waiting for "play".
     **/



    @Override
    public void init_loop() {




    }

    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);



    }

    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {
    }
    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();
    }



    public void Hw_init() {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        Intake = hardwareMap.get(DcMotorEx.class, "Intake");
        flyBot = hardwareMap.get(DcMotorEx.class, "flyBot");
        flyTop = hardwareMap.get(DcMotorEx.class, "flyTop");
        turretSpin = hardwareMap.get(DcMotorEx.class, "turretSpin");

        beamBreaker = hardwareMap.get(DigitalChannel.class, "beamBreaker");


        Pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "Pinpoint");


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());//todo


        Hood = hardwareMap.get(Servo.class, "Hood");
        Blocker = hardwareMap.get(Servo.class, "Blocker");
        Tripod = hardwareMap.get(Servo.class, "Tripod");

        Limelight = hardwareMap.get(Limelight3A.class, "Limelight");

//        imu = hardwareMap.get(IMU.class, "imu");
//        // This needs to be changed to match the orientation on your robot
//        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
//                RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
//        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
//                RevHubOrientationOnRobot.UsbFacingDirection.UP;
//
//        RevHubOrientationOnRobot orientationOnRobot = new
//                RevHubOrientationOnRobot(logoDirection, usbDirection);
//        imu.initialize(new IMU.Parameters(orientationOnRobot));


        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Intake.setVelocity(0);
        Intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flyBot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        flyBot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flyTop.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flyBot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flyTop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        flyBot.setDirection(DcMotorSimple.Direction.REVERSE);
        flyTop.setDirection(DcMotorSimple.Direction.FORWARD);

        Hood.setDirection(Servo.Direction.REVERSE);




        turretSpin.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        turretSpin.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretSpin.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


//        Flylut.add(-13.5,1750); //far
//
//        Flylut.add(-12.7,1700); //far
//
//        Flylut.add(-11.6 , 1550); // far
//
//
//
//
//        Flylut.add(-9.27,1400); //close
//
//
//        Flylut.add(-2.71,1240); //close
//
//        Flylut.add(6.28,1060); //close
//
//        Flylut.add(11 , 1000); // close
//
//
//
//        Hoodlut.add(-13.5,0.78);   //far
//
//        Hoodlut.add(-12.7,0.75);   //far
//
//        Hoodlut.add(-11.6 ,0.7);    //far
//
//
//
//        Hoodlut.add(-9.27,0.55);
//
//        Hoodlut.add(-2.71,0.45);
//
//        Hoodlut.add(6.28,0.2);
//
//        Hoodlut.add(11,0.18);




// far hood pos 0.48 power 0.9

//        Flylut.add(-15, 0.92);  //far 0.89
//
//        Flylut.add(-14, 0.895);  //far 0.87
//        Flylut.add(-13.8, 0.9);  //far 0.85
//
//        Flylut.add(-13, 0.878);  //far 0.85
//        Flylut.add(-12.8, 0.863);  //far   0.82
//        Flylut.add(-11.5, 0.82);  //far   0.82
//
//        Flylut.add(-10.5, 0.77); //+1 // Input camera Ty, Output flywheel power
//        Flylut.add(-9.55, 0.744);   // - 9.55 0.78 (2.0 hood)
//        Flylut.add(-9.00, 0.735);   // - 9.55 0.78 (2.0 hood)
//        Flylut.add(-8.70, 0.73);   // - 9.55 0.78 (2.0 hood)
//        Flylut.add(-6.55, 0.7);// -6.55 0.74
//        Flylut.add(-0.59, 0.63); // - 0.59 0.7
//        Flylut.add(3.65, 0.6); // 3.65 0.67
//
//        Flylut.add(11, 0.58); // 10 0.67 ; 0.0 hood
//
//        // NEAR HOOD ANGLES
//        Hoodlut.add(-10.5, 0.59);  //close
//
////        Hoodlut.add(-9.5 , 0.5);  //close
//
//        Hoodlut.add(-6.5, 0.48);  //close
//
//        Hoodlut.add(-0.65, 0.25);  //close
//        Hoodlut.add(4, 0.19);  //close
//        Hoodlut.add(11, 0);  //close


//        Flylut.createLUT();
//
//        Hoodlut.createLUT();


    }


}