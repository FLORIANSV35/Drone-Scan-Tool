// Adapted from the ExpressLRS Backpack project (lib/MSP/msptypes.h)
// https://github.com/ExpressLRS/Backpack — GPL-3.0-only
// Copyright (c) ExpressLRS contributors
//
// Only the opcodes actually used by the Timer Backpack bridge are kept.
#pragma once

// ELRS specific opcodes
#define MSP_ELRS_BIND                           0x09
#define MSP_ELRS_GET_BACKPACK_VERSION           0x10
#define MSP_ELRS_SET_SEND_UID                   0x00B5

// outgoing, originating from the backpack / forwarded to the VRx
#define MSP_ELRS_BACKPACK_SET_RECORDING_STATE   0x0305

// incoming, originating from the VRx / RotorHazard
#define MSP_ELRS_BACKPACK_SET_MODE              0x0380  // enable binding mode
