/**
 * WallFollowing.h
 * 	Author: Rahul Yarlagadda, Ellie Honious
 * 	Date: December 13th, 2017
 *
 * 	This file contains the basic motor control and odometry code for the robot. Basic functions for driving forward, reverse
 * 	and going to an angle are written.
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

void followWall();


#endif /* WALLFOLLOWING_H_ */
