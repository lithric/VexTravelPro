#pragma once
// ----------------------------------------------------------------------------
//                                                                            
//    Project:                                               
//    Author:
//    Created:
//    Configuration:        
//    Ports: 
//      (Vision Sensor)Eyeball-15
//      (Motor Group)Arm-3R-8F
//      (Drivetrain)Drivetrain-1-11-10-9-5Inert
//      (Digital Out)StickyPiston-A
//
//    make sure to configure controller to drivetrain                                                                        
// ----------------------------------------------------------------------------

#include <cmath>
#include <tgmath.h>
#include <thread>
#include <vector>
#include <map>
#include <typeinfo>
#include <algorithm>
#include "vex.h"

// namespaces and imported methods
namespace Code
{
    using namespace vex;

    // define some useful changes
#define RESERVE(key,val) key = val;auto RESERVE_ACT

#define _USE_MATH_DEFINES

#define anon(EXP) []() -> void{EXP;}
#define func(type,EXP) () -> type{EXP;}
#define opper(a,...) {std::vector<double>{a}, std::vector<double>{__VA_ARGS__}}
#define toggle(key,id) key.pressed( anon(singleAct[id] = true) )

#define setActionVelocity(act,...) set ## act ## Velocity(__VA_ARGS__)
#define actionFor(act,...) act ## For(__VA_ARGS__)

#define printB(a) Brain.Screen.print(a);Brain.Screen.newLine()
#define replaceB(a,b) Brain.Screen.clearLine(a);Brain.Screen.setCursor(a,1);Brain.Screen.print(b)
#define insertB(line,pos,str) Brain.Screen.clearLine(line);Brain.Screen.setCursor(line,pos);Brain.Screen.print(str)
#define clearB(a) Brain.Screen.clearLine(a)
#define __drive(...) opper(0,__VA_ARGS__)
#define __turn(...) opper(1,__VA_ARGS__)
#define __grab(...) opper(2,__VA_ARGS__)
#define __lift(...) opper(3,__VA_ARGS__)
#define __wait(...) opper(4,__VA_ARGS__)
#define __stop(...) opper(5,__VA_ARGS__)
#define __chase(...) opper(6,__VA_ARGS__)
#define __tilt(...) opper(7,__VA_ARGS__)
#define printIf(cond,T,F) anon(if(cond){Brain.Screen.print(T);Brain.Screen.newLine();}else{Brain.Screen.print(F);Brain.Screen.newLine();})

    // assign global variables
        bool debugMode = false;
        std::vector<bool> singleAct = {false,false,false,false,false,false};
        bool isPistonOpen = false;
        int autonMode = 0;
        std::vector<const char*> autonNames = {"Right","Left","RightWin","LeftWin"};
    //
    // Begin project code
    void preAutonomous(void) {
        if (std::find(autonNames.begin(), autonNames.end(), "Nothing") != autonNames.end()) {
        }
        else {
            autonNames.push_back("Nothing");
        }
        Brain.Screen.clearScreen();
        Brain.Screen.print("pre auton code");
        Controller1.ButtonUp.pressed( anon(singleAct[3] = true) );
        Controller1.ButtonUp.released( anon(if(singleAct[3]){
            autonMode = autonMode-1 < 0 ? autonNames.size()-1:autonMode-1;
            int i=0;
            for (auto const& name : autonNames) {
                if(i != autonMode) {
                    insertB(i+3,1,name);
                }
                else {
                    insertB(i+3,5,name);
                }
                i++;
            };
        singleAct[3] = false;}) );
        Controller1.ButtonDown.pressed( anon(singleAct[3] = true) );
        Controller1.ButtonDown.released( anon(if(singleAct[3]){
            autonMode = autonMode+1 >= autonNames.size() ? 0:autonMode+1;
            int i=0;
            for (auto const& name : autonNames) {
                if(i != autonMode) {
                    insertB(i+3,1,name);
                }
                else {
                    insertB(i+3,5,name);
                }
                i++;
            };
        singleAct[3] = false;}) );
        int i=0;
        for (auto const& name : autonNames) {
            if(i != autonMode) {
                insertB(i+3,1,name);
            }   
            else {
                insertB(i+3,5,name);
            }
            i++;
        }
        StickyPiston.set(!isPistonOpen);
        wait(1, seconds);
    }
    class DriveInstructions {
        private:
        public:
        std::vector<std::vector<std::vector<double>>> instructions;
        double speed = 20;
        void start() {
            for (auto const& instruct : instructions) {
                std::vector<double> key = instruct[0];
                std::vector<double> val = instruct[1];
                int lastSeen = 100; // callback number to establish where object was last seen
                int centerFOV = 316/2; // the center of the screen
                int offsetX = 30; // offset to match the center of the bot
                std::map<std::string,int> objectBounds = {{"left",centerFOV+offsetX},{"right",centerFOV-offsetX}};
                switch ((int)key[0]) {
                    case 0: //__drive
                        val.size() > 1 ?
                        (void)Drivetrain.setDriveVelocity(val[1],percent):
                        (void)Drivetrain.setDriveVelocity(speed,percent);
                        std::abs(val[0]) > 0 ?
                        (void)Drivetrain.driveFor(forward,val[0],inches):
                        (void)Drivetrain.drive(forward);
                        Drivetrain.setDriveVelocity(val[0] == 0 && val[2] > 0 ? speed:-speed,percent);
                    break;
                    case 1: //__turn
                        val.size() > 1 ?
                        (void)Drivetrain.setTurnVelocity(val[1],percent):
                        (void)Drivetrain.setTurnVelocity(speed,percent);
                        std::abs(val[0]) > 0 ?
                        (void)Drivetrain.turnFor(right,val[0],degrees):
                        (void)Drivetrain.turn(val[2] > 0 ? right:left);
                        Drivetrain.setTurnVelocity(speed,percent);
                    break;
                    case 2: //__grab
                        StickyPiston.set(val[0] == 1 ? isPistonOpen:!isPistonOpen);
                    break;
                    case 3: //__lift
                        val.size() > 1 ?
                        (void)Arm.setVelocity(val[1],percent):
                        (void)Arm.setVelocity(speed,percent);
                        std::abs(val[0]) > 0 ?
                        (void)Arm.spinFor(forward,val[0] * 7,degrees):
                        (void)Arm.spin(val[2] > 0 ? forward:reverse);
                        Arm.setVelocity(speed,percent);
                    break;
                    case 4: //__wait
                        wait(val[0],msec);
                    break;
                    case 5: //__stop
                        Drivetrain.stop(hold);
                        Arm.stop(hold);
                        //LeftDriveSmart.stop();
                        //RightDriveSmart.stop();
                    break;
                    case 6: //__chase
                        //Drivetrain.drive(forward);
                        //Eyeball.takeSnapshot(Eyeball__REDGOAL); // get data about where object is
                        while(true) {
                            //Eyeball.takeSnapshot(Eyeball__REDGOAL);
                            if (Eyeball.largestObject.exists) {
                                if (Eyeball.largestObject.width < val[0]) {
                                    if (Eyeball.largestObject.centerX > centerFOV + offsetX) {
                                        //LeftDriveSmart.spin(forward);RightDriveSmart.stop();
                                        lastSeen = 1; // to the right
                                    }
                                    else if (Eyeball.largestObject.centerX < centerFOV - offsetX) {
                                        //LeftDriveSmart.stop();RightDriveSmart.spin(forward);
                                        lastSeen = -1; // to the left
                                    }
                                    else {
                                        //LeftDriveSmart.spin(forward);RightDriveSmart.spin(forward);
                                        lastSeen = 0; // straight ahead
                                    }
                                }
                                else {
                                    lastSeen = 100; // no where
                                }
                            }
                            else {
                                lastSeen == 1 ?
                                (void)anon(
                                    //LeftDriveSmart.spin(forward);
                                    //RightDriveSmart.spin(reverse);
                                ):
                                lastSeen == -1 ?
                                (void)anon(
                                    //LeftDriveSmart.spin(reverse);
                                    //RightDriveSmart.spin(forward);
                                ):
                                lastSeen == 0 ?
                                (void)anon(
                                    //LeftDriveSmart.spin(forward);
                                    //RightDriveSmart.spin(forward);
                                ):
                                (void)anon(
                                    //LeftDriveSmart.stop();
                                    //RightDriveSmart.stop();
                                );
                            }
                            wait(0.05,seconds);
                        }
                        Drivetrain.stop();
                    break;
                    case 7: // __tilt()
                        double value3 = val.size() > 2 ? val[2]:100;
                        double conv = std::min(value3/val[0],value3/val[1]);
                        double value1 = val[0]*conv;
                        double value2 = val[1]*conv;
                        //LeftDriveSmart.spin(forward);
                        //RightDriveSmart.spin(forward);
                        //LeftDriveSmart.setVelocity(value1,percent);
                        //RightDriveSmart.setVelocity(value2,percent);
                    break;
                }
            }
        }
        /*
            __tilt(50,10)
        */
    };
    void userControl(void) {
        Brain.Screen.clearScreen();
        // place driver control in this while loop
        Arm.setVelocity(50,percent);
        Fork.setVelocity(100,percent);
            while (true) {
                wait(20, msec);
                while(true) {
                    Controller1.ButtonY.pressed( anon(singleAct[0] = true) );
                    Controller1.ButtonY.released( anon(
                            if(singleAct[0]) {
                                debugMode = !debugMode;
                                singleAct[0] = false;
                                printB(debugMode ? "Debug Mode ON":"Debug Mode OFF");
                            }
                        ));
                    Controller1.ButtonX.pressed( anon(singleAct[1] = true) );
                    Controller1.ButtonX.released( anon(if(singleAct[1] && debugMode){Brain.Screen.clearScreen();singleAct[1] = false;}) );
                    Controller1.ButtonL1.pressed( anon(singleAct[2] = true) );
                    Controller1.ButtonL1.released( anon(if(singleAct[2] && debugMode){printB("hello");singleAct[2] = false;}) );
                    if (!debugMode) {
                        Controller1.ButtonRight.pressed( anon(StickyPiston = !StickyPiston) );
                        Controller1.ButtonL1.pressing() ? Arm.spin(forward):
                        Controller1.ButtonL2.pressing() ? Arm.spin(reverse):
                        Arm.stop(hold);
                        Controller1.ButtonR1.pressing() ? Fork.spin(forward):
                        Controller1.ButtonR2.pressing() ? Fork.spin(reverse):
                        Fork.stop(hold);
                    }

                }
            }
    }
};