#pragma once

namespace Onboard
{
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
