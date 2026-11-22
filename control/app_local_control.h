/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#ifndef APP_LOCAL_CONTROL_H
#define APP_LOCAL_CONTROL_H
#include <stdint.h>
enum { APP_LOCAL_ENABLE, APP_LOCAL_DISABLE, APP_LOCAL_STOP, APP_LOCAL_RESUME,
       APP_LOCAL_FREE, APP_LOCAL_LOCK, APP_LOCAL_JOINT_IMP, APP_LOCAL_CART_IMP, APP_LOCAL_JOINT_ADM,APP_LOCAL_CART_ADM, APP_LOCAL_ACTIONS };
enum { APP_LOCAL_READY, APP_LOCAL_PROFILE, APP_LOCAL_ENABLING, APP_LOCAL_ENABLED,
       APP_LOCAL_DISABLED, APP_LOCAL_STOPPED, APP_LOCAL_RESUMED, APP_LOCAL_REJECTED };
typedef struct { uint8_t action, state, result; uint16_t pending_actions; } AppLocalControlStatus;
/* LCD only posts intentions. All motor operations run in the control task. */
void AppLocalControl_Request(uint8_t action);
uint8_t AppLocalControl_SetupPending(void);
void AppLocalControl_Process(void);
void AppLocalControl_Status(AppLocalControlStatus *out);
const char *AppLocalControl_Message(const AppLocalControlStatus *status);
#endif
