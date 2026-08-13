/*******************************************************************************
 * @file net_stream.hpp
 * @brief WiFi SoftAP + UDP full-rate sample streaming.
 *
 * The bracelet hosts its own network (SSID EMG8-<MAC>, WPA2). A client
 * subscribes by sending any UDP datagram to <bracelet-ip>:3333; the firmware
 * then streams binary packets to that endpoint until another client
 * subscribes or the radio is turned off.
 *
 * Packet layout (little-endian):
 *   offset 0  char[2]  magic "E8"
 *   offset 2  uint8    version (1)
 *   offset 3  uint8    type: 0 = raw Sample[], 1 = env Sample[], 2 = ImuSample[]
 *   offset 4  uint32   per-type sequence number (gaps ⇒ lost packets)
 *   offset 8  uint16   record count
 *   offset 10 uint16   reserved
 *   offset 12 records  count × (8 B Sample or 20 B ImuSample)
 *
 * A partial batch is flushed after 30 ms so viewer latency stays low at
 * envelope/IMU rates. Radio is off until netStreamStart() (UART 'W1').
 ******************************************************************************/
#pragma once

#include "emg8_types.hpp"
#include "esp_err.h"

/** @brief Bring up SoftAP + UDP socket + sender task. Idempotent. */
esp_err_t netStreamStart(const char* macStr);

/** @brief Stop streaming and turn the radio off. Prints #NET stats. */
void netStreamStop();

/** @brief True between netStreamStart() and netStreamStop(). */
bool netStreamActive();

/** @brief Non-blocking enqueue; silently counts drops when the net queue is full. */
void netEnqueueRaw(const Sample& s);
void netEnqueueEnv(const Sample& s);
void netEnqueueImu(const ImuSample& s);

uint32_t netPacketsSent();
uint32_t netDropCount();
