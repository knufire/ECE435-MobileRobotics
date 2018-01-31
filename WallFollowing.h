/**
 * WallFollowing.ino
 * 	Author: Rahul Yarlagadda, Ellie Honious
 * 	Date: December 13th, 2017
 *
 * 	This file contains behaviors for following walls and navigating corners. It also maintains it's own internal state
 * 	machine to execute corner manuvers and continue following.
 */

#pragma once
#ifndef WALLFOLLOWING_H_
#define WALLFOLLOWING_H_

/**
 * State machine varable and states
 */
extern int wallState;   //state to hold robot states and motor motion
#define AVOID     				0   //avoid behavior
#define WANDER    				1   //wander behavior
#define FOLLOW_LEFT     		2   //follow left wall behavior
#define FOLLOW_RIGHT    		3   //follow right wall behavior
#define CENTER    				4   //follow hallway behavior
#define RIGHT_INSIDE_CORNER		5
#define LEFT_INSIDE_CORNER		6
#define RIGHT_OUTSIDE_CORNER 	7
#define LEFT_OUTSIDE_CORNER		8

/**
 * Follows a wall if presesnt, otherwise randomly wanders. Uses blocking drive functions. Directly interfaces with the robot drivetrain.
 */
void followWall();


#endif /* WALLFOLLOWING_H_ */
