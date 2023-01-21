// This file is based on UNSW Sydney's codebase, but has been modified significantly.
// Both copyright notices are provided below.
//
// Copyright (c) 2018 UNSW Sydney.  All rights reserved.
//
// Licensed under Team rUNSWift's original license. See the "LICENSE-runswift"
// file to obtain a copy of the license.
//
// ---------------------------------------------------------------------------------
//
// Copyright 2021 Kenji Brameld
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include "walk/walk.hpp"
#include "walk/maths_functions.hpp"
#include "std_msgs/msg/string.hpp"
// #include "walk/bodyV5.hpp"
#include "walk/bodyV6.hpp"
#include "walk/MotionDefs.hpp"
#include "walk/XYZ_Coord.hpp"


#define KICK_STEP_HEIGHT 0.065  // how far to lift kicking foot

namespace Joints = V6Joints;
namespace Limbs = V6Limbs;
namespace LEDs = V6LEDs;
namespace Sensors = V6Sensors;


#define EPSILON 0.01       //10 mm
#define TURN_EPSILON 0.05  //2.8 degrees

#define MAX_EXTRA_LEAN 1  // degrees
#define KICK_MIN_Y_DIST 30 //mm
#define KICK_MAX_Y_DIST 200 //mm

#define DEFAULT_ARM_STIFFNESS 0.1 // stiffness of arms joints during walk on default

#define BIG_NUM 1000000000000.0

// using boost::program_options::variables_map;
// using namespace std;
// using namespace Joints;
// using namespace Sensors;

const float MM_PER_M = 1000.0;             // number of millimeters in one meter
const float CROUCH_STAND_PERIOD = 0.5;              // time in seconds to crouch
const float COM_OFFSET_CROUCH = 0.01; // center of mass offset in x direction in meters when crouched, so that the CoM lies at the centre of the heel and toe
const float COM_OFFSET_FORWARDS = 0.022;   // center of mass offset in x direction in meters when walking forwards
const float COM_OFFSET_BACKWARDS = 0.01;   // center of mass offset in x direction in meters when walking backwards
const float FORWARD_CHANGE = 0.06; // max change of 80mm/sec at each leg change
const float LEFT_CHANGE = 0.1; // max change of 100mm/sec at each leg change
const float TURN_CHANGE = 1.0; // max change of 1.0rad/sec at each leg change (only when forward < MAX_FORWARD_TURN_CHANGE_SLOW)
const float TURN_CHANGE_SLOW = 0.5;     // when forward > MAX_FORWARD_RESTRICT_TURN_CHANGE
const float MAX_FORWARD_TURN_CHANGE_SLOW = 0.1; // When forward is greater than this value lower TURN_CHANGE to TURN_CHANGE_STABLE
const float STAND_HIP_HEIGHT = 0.248; // for tall power saving stand, matches INITIAL action command
const float KNEE_PITCH_RANGE = DEG2RAD(60); // the knee pitch range from standing to crouching
const float BASE_WALK_PERIOD = .25;    //.23  - .25            // seconds to walk one step
const float WALK_HIP_HEIGHT = .23; // Walk hip height - seems to work from .2 to .235
// simulated robot falls over at higher walk speeds (TODO try to debug this problem)
const float MAX_FORWARD = .3;                              // meters
const float MAX_LEFT = .2;                                 // meters
const float MAX_TURN = 2.0;                                // radians
const float BASE_LEG_LIFT = 0.012;                         // meters
const float MAX_LEFT_BLOCKING = .4;                        // meters
const float z = 0;

float evaluateWalkVolume(float x, float y, float z);


Walk::Walk(
  std::function<void(void)> notifyWalkDone,
  std::function<void(biped_interfaces::msg::SolePoses)> sendSolePoses,
  rclcpp::Node* walkNode)
: notifyWalkDone(notifyWalkDone), sendSolePoses(sendSolePoses), walkNode(walkNode), t(0.0f), weightHasShifted(false)
{
  
}

void Walk::start(walk_msg::msg::Walk walk_command)
{
  
  dt = MOTION_DT;
  t = 0.0;                                   // initialise timers (in seconds)
  timer = 0.0;                            // timer to crouch to walking height
  globalTime = 0;                          // use for diagnostic purposes only
  T = BASE_WALK_PERIOD; // seconds - the period of one step
  stopping = false;                         // legacy code for stopping robot?
  stopped = true;                            // legacy code for stopped robot?
  leftL = leftR = lastLeft = left = 0.0; // Side-step for left, right foot, and (last) left command in meters
  turnRL = turnRL0 = lastTurn = turn = 0.0;       // Initial turn variables for feet
  forwardL = forwardR = 0.0; // forward step per for left and right foot in meters
  forwardR0 = forwardL0 = 0; // initial last positions for left and right feet keep constant during next walk step
  forward = lastForward = 0.0;           // Current and previous forward value
  shoulderPitchL = shoulderPitchR = 0;              // arm swing while walking
  shoulderRollL = shoulderRollR = 0;                   // arm roll during kick
  hiph = hiph0 = STAND_HIP_HEIGHT; // make robot stand initially based on Stand command
  foothL = foothR = 0;         // robots feet are both on the ground initially
  thigh = Limbs::ThighLength / MM_PER_M;             // thigh length in meters
  tibia = Limbs::TibiaLength / MM_PER_M;             // tibia length in meters
  ankle = Limbs::FootHeight / MM_PER_M;        // height of ankle above ground
  nextFootSwitchT = 0.0; // next time-point to switch support foot (in seconds)
  stiffness = kneeStiffness = ankleStiffness = 0.9; // initial motor stiffness
  currentVolume = 0;                      // initial volume
  walk2014Option = NONE;                           // initial walk 2014 option
  walkState = NOT_WALKING;                               // initial walkState
  supportFoothasChanged = false;       // triggers support foot change actions
  comOffset = targetComOffset = comOffset0 = 0; // Center of Mass offset in sagittal plane used to spread weight along feet in x-dir
  prevTurn = prevForwardL = prevForwardR = 0;            // odometry
  prevLeftL = prevLeftR = 0;                             // odometry
  exactStepsRequested = false; // turns off ratcheting as requested by WalkEnginePreProcessor for kick
  shouldEmergencyStep = false; // Whether robot sohuld take an emergency step
  currentTurnChange = TURN_CHANGE;

  // Balance control
  filteredGyroX = 0;
  filteredGyroY = 0;
  filteredAngleY = 0;
  sagittalBalanceAdjustment = 0;
  coronalBalanceAdjustment = 0;
  sagittalHipBalanceAdjustment = 0;

  // Gyro PD controller
  preErrorGyro = 0;             // Previous tick gyro error

  // Angle PID controller
  angleError = 0;               // Current angle error
  angleErrorSum = 0;            // Error sum
  preErrorAngle = 0;            // Previous tick angle error

  // Kick specific
  kickT = 0;
  stableCounter = 0;
  rock = 0;
  kneePitchL = kneePitchR = lastKneePitch = 0;
  anklePitchL = anklePitchR = 0;
  lastKickForward = 0;
  lastSide = 0;
  lastKickTime = T;
  dynamicSide = 0.0f;
  turnAngle = 0;
  lastKickTurn = 0;
  //motionOdometry.reset();
  kneePitchEnd = 0;
  anklePitchStart = 0;
  anklePitchEnd = 0;
  swingDelayFactor = 0;
  holdAnkle = false;
  hipBalance = false;
  ballXYDebug = false;

  timerSinceLastBlock = 0;
  

}

void Walk::notifyJoints(walk_sensor_msg::msg::Sensor sensor_readings)
{
  
  
}



float ellipsoidClampWalk(float &forward, float &left, float &turn, float speed) {
    const float MIN_SPEED = 0.0f;
    const float MAX_SPEED = 1.0f;
    speed = crop(speed, MIN_SPEED, MAX_SPEED);

    // limit max to 66-100% depending on speed
    float M_FORWARD = MAX_FORWARD * 0.66 + MAX_FORWARD * 0.34 * speed;
    float M_LEFT = MAX_LEFT * 0.66 + MAX_LEFT * 0.34 * speed;
    float M_TURN = MAX_TURN * 0.66 + MAX_TURN * 0.34 * speed;

    // Values in range [-1..1]
    float forwardAmount = forward / M_FORWARD;
    float leftAmount = left / M_LEFT;
    float turnAmount = turn / M_TURN;

    float x = fabs(forwardAmount);
    float y = fabs(leftAmount);
    float z = fabs(turnAmount);

    // see if the point we are given is already inside the allowed walk params volume
    if (evaluateWalkVolume(x, y, z) > 1.0) {
        float scale = 0.5;
        float high = 1.0;
        float low = 0.0;

        // This is basically a binary search to find the point on the surface.
        for (unsigned i = 0; i < 10; i++) {
            x = fabs(forwardAmount) * scale;
            y = fabs(leftAmount) * scale;
            z = fabs(turnAmount) * scale;

            if (evaluateWalkVolume(x, y, z) > 1.0) {
                float newScale = (scale + low) / 2.0;
                high = scale;
                scale = newScale;
            } else {
                float newScale = (scale + high) / 2.0;
                low = scale;
                scale = newScale;
            }
        }

        forwardAmount *= scale;
        leftAmount *= scale;
        turnAmount *= scale;
    }

    forward = M_FORWARD * forwardAmount;
    left = M_LEFT * leftAmount;
    turn = M_TURN * turnAmount;
    float volume = evaluateWalkVolume(x, y, z);
    return volume;
}

// x = forward, y = left, z = turn
float evaluateWalkVolume(float x, float y, float z) {
    return sqrt(x * x + y * y + z * z);
}



float Walk::parabolicReturn(float f) { //normalised [0,1] up and down
    double x = 0;
    double y = 0;
    if (f < 0.25f) {
        y = 8 * f * f;
    }
    if (f >= 0.25f && f < 0.5f) {
        x = 0.5f - f;
        y = 8 * x * x;
        y = 1.0f - y;
    }
    if (f >= 0.5f && f < 0.75f) {
        x = f - 0.5f;
        y = 8 * x * x;
        y = 1.0f - y;
    }
    if (f >= 0.75f && f <= 1.0f) {
        x = 1.0f - f;
        y = 8 * x * x;
    }
    return y;
}

float Walk::parabolicReturnMod(float f) { //normalised [0,1] up and down
    double x = 0;
    double y = 0;
    if (f < 0.25f) {
        // y: 0 -> 0.75
        y = 8 * f * f * 1.50;
    }
    if (f >= 0.25f && f < 0.5f) {
        // y: 0.75 -> 1.00
        x = 0.5f - f;
        y = 8 * x * x;
        y = y / 2;
        y = 1.0f - y;
    }
    if (f >= 0.5f && f < 0.75f) {
        // y: 1.00 -> 0.75
        x = f - 0.5f;
        y = 8 * x * x;
        y = y / 2;
        y = 1.0f - y;
    }
    if (f >= 0.75f && f <= 1.0f) {
        // y: 0.75 -> 0
        x = 1.0f - f;
        y = 8 * x * x * 1.50;
    }
    return y;
}

float Walk::parabolicStep(float time, float period, float deadTimeFraction) { //normalised [0,1] step up
    float deadTime = period * deadTimeFraction / 2;
    if (time < deadTime + dt / 2)
        return 0;
    if (time > period - deadTime - dt / 2)
        return 1;
    float timeFraction = (time - deadTime) / (period - 2 * deadTime);
    if (time < period / 2)
        return 2.0 * timeFraction * timeFraction;
    return 4 * timeFraction - 2 * timeFraction * timeFraction - 1;
}

float Walk::linearStep(float time, float period) {
    if (time <= 0)
        return 0;
    if (time >= period)
        return 1;
    return time / period;
}


float Walk::interpolateSmooth(float start, float end, float tCurrent, float tEnd) {
    return start + (end - start) * (1 + cos(M_PI * tCurrent / tEnd - M_PI)) / 2;
}

float Walk::squareSmooth(float start, float end, float tCurrent, float tEnd) {
    return ((end - start)/(tEnd * tEnd)) * ((tCurrent * tCurrent)) + start;
}

XYZ_Coord Walk::mf2b(float Hyp, float Hp, float Hr, float Kp, float Ap, float Ar, float xf, float yf, float zf) {
    // MFOOT2BODY Transform coords from foot to body.
    // This code originates from 2010 using symbolic equations in Matlab to perform the coordinate transforms - see team report (BH)
    // In future this approach to IK for the Nao should be reworked in closed form, significantly reducing the size of the code the
    // the computational complexity (BH)
    XYZ_Coord result;
    float pi = M_PI;
    float tibia = this->tibia * 1000;
    float thigh = this->thigh * 1000;
    float k = sqrt(2.0);
    float c1 = cos(Ap);
    float c2 = cos(Hr + pi / 4.0);
    float c3 = cos(Hyp - pi / 2.0);
    float c4 = cos(Hp);
    float c5 = cos(Kp);
    float c6 = cos(Ar - pi / 2.0);
    float s1 = sin(Kp);
    float s2 = sin(Hp);
    float s3 = sin(Hyp - 1.0 / 2.0 * pi);
    float s4 = sin(Hr + 1.0 / 4.0 * pi);
    float s5 = sin(Ap);
    float s6 = sin(Ar - 1.0 / 2.0 * pi);
    result.x = thigh * (s2 * s3 - c2 * c3 * c4) + tibia * (s1 * (c4 * s3 + c2 * c3 * s2) + c5 * (s2 * s3 - c2 * c3 * c4))
            - yf
                    * (c6 * (c1 * (s1 * (c4 * s3 + c2 * c3 * s2) + c5 * (s2 * s3 - c2 * c3 * c4)) - s5 * (s1 * (s2 * s3 - c2 * c3 * c4) - c5 * (c4 * s3 + c2 * c3 * s2)))
                            + c3 * s4 * s6)
            + zf
                    * (s6 * (c1 * (s1 * (c4 * s3 + c2 * c3 * s2) + c5 * (s2 * s3 - c2 * c3 * c4)) - s5 * (s1 * (s2 * s3 - c2 * c3 * c4) - c5 * (c4 * s3 + c2 * c3 * s2)))
                            - c3 * c6 * s4)
            + xf * (c1 * (s1 * (s2 * s3 - c2 * c3 * c4) - c5 * (c4 * s3 + c2 * c3 * s2)) + s5 * (s1 * (c4 * s3 + c2 * c3 * s2) + c5 * (s2 * s3 - c2 * c3 * c4)));
    result.y = xf
            * (c1 * (c5 * (s2 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) - (c3 * c4 * k) / 2.0f) + s1 * (c4 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) + (c3 * k * s2) / 2.0f))
                    + s5
                            * (c5 * (c4 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) + (c3 * k * s2) / 2.0f)
                                    - s1 * (s2 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) - (c3 * c4 * k) / 2.0f)))
            + tibia * (c5 * (c4 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) + (c3 * k * s2) / 2.0f) - s1 * (s2 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) - (c3 * c4 * k) / 2.0f))
            + thigh * (c4 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) + (c3 * k * s2) / 2.0f)
            - yf
                    * (s6 * ((c2 * k) / 2.0f - (k * s3 * s4) / 2.0f)
                            + c6
                                    * (c1
                                            * (c5 * (c4 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) + (c3 * k * s2) / 2.0f)
                                                    - s1 * (s2 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) - (c3 * c4 * k) / 2.0f))
                                            - s5
                                                    * (c5 * (s2 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) - (c3 * c4 * k) / 2.0f)
                                                            + s1 * (c4 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) + (c3 * k * s2) / 2.0f))))
            - zf
                    * (c6 * ((c2 * k) / 2.0f - (k * s3 * s4) / 2.0f)
                            - s6
                                    * (c1
                                            * (c5 * (c4 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) + (c3 * k * s2) / 2.0f)
                                                    - s1 * (s2 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) - (c3 * c4 * k) / 2.0f))
                                            - s5
                                                    * (c5 * (s2 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) - (c3 * c4 * k) / 2.0f)
                                                            + s1 * (c4 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) + (c3 * k * s2) / 2.0f))));
    result.z = yf
            * (s6 * ((c2 * k) / 2.0f + (k * s3 * s4) / 2.0f)
                    + c6
                            * (c1
                                    * (c5 * (c4 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) - (c3 * k * s2) / 2.0f)
                                            - s1 * (s2 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) + (c3 * c4 * k) / 2.0f))
                                    - s5
                                            * (c5 * (s2 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) + (c3 * c4 * k) / 2.0f)
                                                    + s1 * (c4 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) - (c3 * k * s2) / 2.0f))))
            - tibia * (c5 * (c4 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) - (c3 * k * s2) / 2.0f) - s1 * (s2 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) + (c3 * c4 * k) / 2.0f))
            - thigh * (c4 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) - (c3 * k * s2) / 2.0f)
            - xf
                    * (c1
                            * (c5 * (s2 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) + (c3 * c4 * k) / 2.0f)
                                    + s1 * (c4 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) - (c3 * k * s2) / 2.0f))
                            + s5
                                    * (c5 * (c4 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) - (c3 * k * s2) / 2.0f)
                                            - s1 * (s2 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) + (c3 * c4 * k) / 2.0f)))
            + zf
                    * (c6 * ((c2 * k) / 2.0f + (k * s3 * s4) / 2.0f)
                            - s6
                                    * (c1
                                            * (c5 * (c4 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) - (c3 * k * s2) / 2.0f)
                                                    - s1 * (s2 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) + (c3 * c4 * k) / 2.0f))
                                            - s5
                                                    * (c5 * (s2 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) + (c3 * c4 * k) / 2.0f)
                                                            + s1 * (c4 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) - (c3 * k * s2) / 2.0f))));
    return result;
}

Walk::Hpr Walk::hipAngles(float Hyp, float Hp, float Hr, float Kp, float Ap, float Ar, float xf, float yf, float zf, XYZ_Coord e) {
    // Code from 2010 to perform interative Inverse Kinematics.
    // Symbolic equations generated in Matlab - see 2010 team report for details and reference
    Hpr result;
    float pi = M_PI;
    float tibia = this->tibia * 1000;
    float thigh = this->thigh * 1000;
    float k = sqrt(2.0);
    float c1 = cos(Ap);
    float c2 = cos(Hr + pi / 4.0);
    float c3 = cos(Hyp - pi / 2.0);
    float c4 = cos(Hp);
    float c5 = cos(Kp);
    float c6 = cos(Ar - pi / 2.0);
    float s1 = sin(Kp);
    float s2 = sin(Hp);
    float s3 = sin(Hyp - 1.0 / 2.0 * pi);
    float s4 = sin(Hr + 1.0 / 4.0 * pi);
    float s5 = sin(Ap);
    float s6 = sin(Ar - 1.0 / 2.0 * pi);
    float j11 = thigh * (c4 * s3 + c2 * c3 * s2) - tibia * (s1 * (s2 * s3 - c2 * c3 * c4) - c5 * (c4 * s3 + c2 * c3 * s2))
            + xf * (c1 * (s1 * (c4 * s3 + c2 * c3 * s2) + c5 * (s2 * s3 - c2 * c3 * c4)) - s5 * (s1 * (s2 * s3 - c2 * c3 * c4) - c5 * (c4 * s3 + c2 * c3 * s2)))
            + c6 * yf * (c1 * (s1 * (s2 * s3 - c2 * c3 * c4) - c5 * (c4 * s3 + c2 * c3 * s2)) + s5 * (s1 * (c4 * s3 + c2 * c3 * s2) + c5 * (s2 * s3 - c2 * c3 * c4)))
            - s6 * zf * (c1 * (s1 * (s2 * s3 - c2 * c3 * c4) - c5 * (c4 * s3 + c2 * c3 * s2)) + s5 * (s1 * (c4 * s3 + c2 * c3 * s2) + c5 * (s2 * s3 - c2 * c3 * c4)));
    float j12 = yf * (c6 * (c1 * (c3 * s1 * s2 * s4 - c3 * c4 * c5 * s4) + s5 * (c3 * c4 * s1 * s4 + c3 * c5 * s2 * s4)) - c2 * c3 * s6)
            - tibia * (c3 * s1 * s2 * s4 - c3 * c4 * c5 * s4)
            - zf * (s6 * (c1 * (c3 * s1 * s2 * s4 - c3 * c4 * c5 * s4) + s5 * (c3 * c4 * s1 * s4 + c3 * c5 * s2 * s4)) + c2 * c3 * c6)
            + xf * (c1 * (c3 * c4 * s1 * s4 + c3 * c5 * s2 * s4) - s5 * (c3 * s1 * s2 * s4 - c3 * c4 * c5 * s4)) + c3 * c4 * s4 * thigh;
    float j21 = xf
            * (c1 * (c5 * (c4 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) + (c3 * k * s2) / 2.0f) - s1 * (s2 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) - (c3 * c4 * k) / 2.0f))
                    - s5
                            * (c5 * (s2 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) - (c3 * c4 * k) / 2.0f)
                                    + s1 * (c4 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) + (c3 * k * s2) / 2.0f)))
            - tibia * (c5 * (s2 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) - (c3 * c4 * k) / 2.0f) + s1 * (c4 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) + (c3 * k * s2) / 2.0f))
            - thigh * (s2 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) - (c3 * c4 * k) / 2.0f)
            + c6 * yf
                    * (c1
                            * (c5 * (s2 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) - (c3 * c4 * k) / 2.0f)
                                    + s1 * (c4 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) + (c3 * k * s2) / 2.0f))
                            + s5
                                    * (c5 * (c4 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) + (c3 * k * s2) / 2.0f)
                                            - s1 * (s2 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) - (c3 * c4 * k) / 2.0f)))
            - s6 * zf
                    * (c1
                            * (c5 * (s2 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) - (c3 * c4 * k) / 2.0f)
                                    + s1 * (c4 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) + (c3 * k * s2) / 2.0f))
                            + s5
                                    * (c5 * (c4 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) + (c3 * k * s2) / 2.0f)
                                            - s1 * (s2 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) - (c3 * c4 * k) / 2.0f)));
    float j22 = tibia * (c4 * c5 * ((c2 * k) / 2.0f - (k * s3 * s4) / 2.0f) - s1 * s2 * ((c2 * k) / 2.0f - (k * s3 * s4) / 2.0f))
            + xf
                    * (c1 * (c4 * s1 * ((c2 * k) / 2.0f - (k * s3 * s4) / 2.0f) + c5 * s2 * ((c2 * k) / 2.0f - (k * s3 * s4) / 2.0f))
                            + s5 * (c4 * c5 * ((c2 * k) / 2.0f - (k * s3 * s4) / 2.0f) - s1 * s2 * ((c2 * k) / 2.0f - (k * s3 * s4) / 2.0f)))
            + yf
                    * (s6 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f)
                            - c6
                                    * (c1 * (c4 * c5 * ((c2 * k) / 2.0f - (k * s3 * s4) / 2.0f) - s1 * s2 * ((c2 * k) / 2.0f - (k * s3 * s4) / 2.0f))
                                            - s5 * (c4 * s1 * ((c2 * k) / 2.0f - (k * s3 * s4) / 2.0f) + c5 * s2 * ((c2 * k) / 2.0f - (k * s3 * s4) / 2.0f))))
            + zf
                    * (c6 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f)
                            + s6
                                    * (c1 * (c4 * c5 * ((c2 * k) / 2.0f - (k * s3 * s4) / 2.0f) - s1 * s2 * ((c2 * k) / 2.0f - (k * s3 * s4) / 2.0f))
                                            - s5 * (c4 * s1 * ((c2 * k) / 2.0f - (k * s3 * s4) / 2.0f) + c5 * s2 * ((c2 * k) / 2.0f - (k * s3 * s4) / 2.0f))))
            + c4 * thigh * ((c2 * k) / 2.0f - (k * s3 * s4) / 2.0f);
    float j31 = tibia * (c5 * (s2 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) + (c3 * c4 * k) / 2.0f) + s1 * (c4 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) - (c3 * k * s2) / 2.0f))
            - xf
                    * (c1
                            * (c5 * (c4 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) - (c3 * k * s2) / 2.0f)
                                    - s1 * (s2 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) + (c3 * c4 * k) / 2.0f))
                            - s5
                                    * (c5 * (s2 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) + (c3 * c4 * k) / 2.0f)
                                            + s1 * (c4 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) - (c3 * k * s2) / 2.0f)))
            + thigh * (s2 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) + (c3 * c4 * k) / 2.0f)
            - c6 * yf
                    * (c1
                            * (c5 * (s2 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) + (c3 * c4 * k) / 2.0f)
                                    + s1 * (c4 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) - (c3 * k * s2) / 2.0f))
                            + s5
                                    * (c5 * (c4 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) - (c3 * k * s2) / 2.0f)
                                            - s1 * (s2 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) + (c3 * c4 * k) / 2.0f)))
            + s6 * zf
                    * (c1
                            * (c5 * (s2 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) + (c3 * c4 * k) / 2.0f)
                                    + s1 * (c4 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) - (c3 * k * s2) / 2.0f))
                            + s5
                                    * (c5 * (c4 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) - (c3 * k * s2) / 2.0f)
                                            - s1 * (s2 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) + (c3 * c4 * k) / 2.0f)));
    float j32 = -tibia * (c4 * c5 * ((c2 * k) / 2.0f + (k * s3 * s4) / 2.0f) - s1 * s2 * ((c2 * k) / 2.0f + (k * s3 * s4) / 2.0f))
            - xf
                    * (c1 * (c4 * s1 * ((c2 * k) / 2.0f + (k * s3 * s4) / 2.0f) + c5 * s2 * ((c2 * k) / 2.0f + (k * s3 * s4) / 2.0f))
                            + s5 * (c4 * c5 * ((c2 * k) / 2.0f + (k * s3 * s4) / 2.0f) - s1 * s2 * ((c2 * k) / 2.0f + (k * s3 * s4) / 2.0f)))
            - yf
                    * (s6 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f)
                            - c6
                                    * (c1 * (c4 * c5 * ((c2 * k) / 2.0f + (k * s3 * s4) / 2.0f) - s1 * s2 * ((c2 * k) / 2.0f + (k * s3 * s4) / 2.0f))
                                            - s5 * (c4 * s1 * ((c2 * k) / 2.0f + (k * s3 * s4) / 2.0f) + c5 * s2 * ((c2 * k) / 2.0f + (k * s3 * s4) / 2.0f))))
            - zf
                    * (c6 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f)
                            + s6
                                    * (c1 * (c4 * c5 * ((c2 * k) / 2.0f + (k * s3 * s4) / 2.0f) - s1 * s2 * ((c2 * k) / 2.0f + (k * s3 * s4) / 2.0f))
                                            - s5 * (c4 * s1 * ((c2 * k) / 2.0f + (k * s3 * s4) / 2.0f) + c5 * s2 * ((c2 * k) / 2.0f + (k * s3 * s4) / 2.0f))))
            - c4 * thigh * ((c2 * k) / 2.0f + (k * s3 * s4) / 2.0f);
    float xbe = e.x;
    float ybe = e.y;
    float zbe = e.z;
    float lambda = 0.4f;
    float la2 = lambda * lambda;
    float la4 = la2 * la2;
    float j322 = j32 * j32;
    float j222 = j22 * j22;
    float j122 = j12 * j12;
    float j212 = j21 * j21;
    float j112 = j11 * j11;
    float j312 = j31 * j31;
    float sigma = 1.0f
            / (la4 + j112 * j222 + j122 * j212 + j112 * j322 + j122 * j312 + j212 * j322 + j222 * j312 + j112 * la2 + j122 * la2 + j212 * la2 + j222 * la2 + j312 * la2 + j322 * la2
                    - 2.0f * j11 * j12 * j21 * j22 - 2.0f * j11 * j12 * j31 * j32 - 2.0f * j21 * j22 * j31 * j32);
    result.Hp = sigma * xbe * (j11 * j222 + j11 * j322 + j11 * la2 - j12 * j21 * j22 - j12 * j31 * j32)
            + sigma * ybe * (j122 * j21 + j21 * j322 + j21 * la2 - j11 * j12 * j22 - j22 * j31 * j32)
            + sigma * zbe * (j122 * j31 + j222 * j31 + j31 * la2 - j11 * j12 * j32 - j21 * j22 * j32);
    result.Hr = sigma * xbe * (j12 * j212 + j12 * j312 + j12 * la2 - j11 * j21 * j22 - j11 * j31 * j32)
            + sigma * ybe * (j112 * j22 + j22 * j312 + j22 * la2 - j11 * j12 * j21 - j21 * j31 * j32)
            + sigma * zbe * (j112 * j32 + j212 * j32 + j32 * la2 - j11 * j12 * j31 - j21 * j22 * j31);
    return result;
}