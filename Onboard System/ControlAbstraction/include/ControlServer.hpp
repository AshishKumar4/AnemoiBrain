#pragma once

namespace Onboard
{
/* ------------------------------------------------------------------------------------------------------------------------ */
/* --------------------------------------------------Some Configurations--------------------------------------------------- */
/* ------------------------------------------------------------------------------------------------------------------------ */

/*
  STREAM_PROTOCOL_1 Refers to conventional failproof streaming of values as packets, encapsulated and later stripped off on the other hand 
  STREAM_PROTOCOL_2 Refers to newer, faster and lightweight but simplest streaming, stream of simple bytes. This isn't failproof and errors 
                    may be entroduced.
*/
//#define STREAM_PROTOCOL_1 // FailProof, Encapsulate
#define STREAM_PROTOCOL_2 // Simple Byte Stream, Faster and lightweight

//#define DRONELESS_LOCAL_TEST

/* ------------------------------------------------------------------------------------------------------------------------ */
/* ------------------------------------------------Permanent Configurations------------------------------------------------ */
/* ------------------------------------------------------------------------------------------------------------------------ */

#if defined(STREAM_PROTOCOL_1)
#undef STREAM_PROTOCOL_2
#elif defined(STREAM_PROTOCOL_2)
#undef STREAM_PROTOCOL_1
#endif

/* ------------------------------------------------------------------------------------------------------------------------ */
/* -----------------------------------------------------Main Program------------------------------------------------------- */
/* ------------------------------------------------------------------------------------------------------------------------ */

namespace Controls
{

const char HANDSHAKE_IN_MSG[] = "Hello Gardien!";
const char HANDSHAKE_OUT_MSG[] = "Hello Overloard!";

int ControlHandshake(int i, int fd);
int ControlListeners(int i, int fd);

} // namespace Controls

#if defined(MSP_SERIAL_FORWARDING)

#include "LowLevel/Serial.hpp"

namespace SerialForwarding
{
void InitializerServer(int i);
int ControlListeners(int i, int fd);
int ControlWriters(int i, int fd);
int MSP_Forward(int i, int fd);
int Handshake(int i, int j);
} // namespace SerialForwarding

namespace RemoteTweaker
{
int RemotePIDChange(int i, int fd);
int Handshake(int i, int j);
} // namespace RemoteTweaker

#endif

int ControlServer_init(int argc, char **argv);
} // namespace Onboard
