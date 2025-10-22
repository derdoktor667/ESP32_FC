#ifndef MSP_COMMANDS_H
#define MSP_COMMANDS_H

// Custom MSP commands for Flight Controller settings
#define MSP_FC_GET_SETTING 2000
#define MSP_FC_SET_SETTING 2001
#define MSP_FC_SETTING_RESPONSE 2002
#define MSP_FC_ERROR 2003
#define MSP_FC_GET_RX_MAP 2004
#define MSP_FC_SET_RX_MAP 2005

// Custom MSP commands for System and Utility
#define MSP_FC_SAVE_SETTINGS 2006
#define MSP_FC_RESET_SETTINGS 2007
#define MSP_FC_REBOOT 2008
#define MSP_FC_GET_STATUS 2009
#define MSP_FC_GET_VERSION 2010
#define MSP_FC_CALIBRATE_IMU 2011

// Custom MSP command for Live Data Streaming
#define MSP_FC_LIVE_DATA 2012

#endif // MSP_COMMANDS_H