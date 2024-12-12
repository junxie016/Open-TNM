/********************************************************
** Copyright 2000 Earth Resource Mapping Ltd.
** This document contains proprietary source code of
** Earth Resource Mapping Ltd, and can only be used under
** one of the three licenses as described in the 
** license.txt file supplied with this distribution. 
** See separate license.txt file for license details 
** and conditions.
**
** This software is covered by US patent #6,442,298,
** #6,102,897 and #6,633,688.  Rights to use these patents 
** is included in the license agreements.
**
** FILE:   	NCSUtil\NCSServerState.h
** CREATED:	06Jul00
** AUTHOR: 	Nicholas Yue
** PURPOSE:	Determines the state of the server
** EDITS:
 *******************************************************/

#ifdef __cplusplus
extern "C" {
#endif

#ifndef SERVERSTATE_H
#define SERVERSTATE_H

extern NCSError  NCSIsWebServerRunning(DWORD nServerID,BOOLEAN *bServerRunning);

#endif // SERVERSTATE_H

#ifdef __cplusplus
}
#endif
