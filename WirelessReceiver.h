/**
 * WirelessReceiver.h
 * 	Author: Rahul Yarlagadda, Ellie Honious
 * 	Date: December 13th, 2017
 *
 * 	This file contains code necessary for receiving data from the laptop.
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





