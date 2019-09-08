#pragma once

#if defined(CLI_MONITOR)

typedef int (*func_t)(); // function pointer
func_t KeyMap[256];
int show_RC = 0, show_PID = 0, show_IMU = 0, show_Wifi = 0, show_armed = 1, show_VELOCITY = 0, show_POSITION = 0;


void Channel_ViewRefresh(int threadId);
int event_key_A();
int event_key_r();
int event_key_q();
int event_key_w();
int event_key_e();
int event_key_P();
int event_key_V();
int event_key_h();
int event_key_other();
int Keyboard_handler(int id);

#endif