/**
 * WallFollowing.ino
 * 	Author: Rahul Yarlagadda, Ellie Honious
 * 	Date: December 13th, 2017
 *
 * 	This file contains behaviors for following walls and navigating corners. It also maintains it's own internal state
 * 	machine to execute corner manuvers and continue following.
 */
#pragma once
#ifndef WIRELESSRECEIVER_H_
#define WIRELESSRECEIVER_H_

#define TEAM_CHANNEL 37

#include "Arduino.h"

void wirelessSetup();
char* wirelessRecieve();
void wirelessSend(String str);

#endif /* WIRELESSRECEIVER_H_ */





