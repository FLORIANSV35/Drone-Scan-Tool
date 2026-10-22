// Adapted from the ExpressLRS Backpack project (lib/MSP/msp.cpp)
// https://github.com/ExpressLRS/Backpack — GPL-3.0-only
// Copyright (c) ExpressLRS contributors
#include "msp.h"
#include "crc.h"

/* ==========================================
MSP V2 Message Structure:
Offset: Usage:         In CRC:  Comment:
======= ======         =======  ========
0       $                       Framing magic start char
1       X                       'X' in place of v1 'M'
2       type                    '<' / '>' / '!' Message Type
3       flag           +        uint8, flag (set to zero)
4       function       +        uint16 (little endian)
6       payload size   +        uint16 (little endian)
8       payload        +        n bytes
n+8     checksum                uint8, crc8_dvb_s2 checksum
========================================== */

static uint8_t crc8_dvb_s2(uint8_t crc, unsigned char a)
{
    static GENERIC_CRC8 crc8_dvb_s2_instance(0xD5);
    return crc8_dvb_s2_instance.calc(crc ^ a);
}

MSP::MSP() : m_inputState(MSP_IDLE)
{
}

bool
MSP::processReceivedByte(uint8_t c)
{
    switch (m_inputState) {

        case MSP_IDLE:
            if (c == '$') {
                m_inputState = MSP_HEADER_START;
            }
            break;

        case MSP_HEADER_START:
            switch (c) {
                case 'X':
                    m_inputState = MSP_HEADER_X;
                    break;
                default:
                    m_inputState = MSP_IDLE;
                    break;
            }
            break;

        case MSP_HEADER_X:
            m_inputState = MSP_HEADER_V2_NATIVE;
            m_packet.reset();
            m_offset = 0;
            m_crc = 0;

            switch (c) {
                case '<':
                    m_packet.type = MSP_PACKET_COMMAND;
                    break;
                case '>':
                    m_packet.type = MSP_PACKET_RESPONSE;
                    break;
                default:
                    m_packet.type = MSP_PACKET_UNKNOWN;
                    m_inputState = MSP_IDLE;
                    break;
            }
            break;

        case MSP_HEADER_V2_NATIVE:
            m_inputBuffer[m_offset++] = c;
            m_crc = crc8_dvb_s2(m_crc, c);

            if (m_offset == sizeof(mspHeaderV2_t)) {
                mspHeaderV2_t* header = (mspHeaderV2_t*)&m_inputBuffer[0];
                m_packet.payloadSize = header->payloadSize;
                m_packet.function = header->function;
                m_packet.flags = header->flags;
                m_offset = 0;
                if (m_packet.payloadSize == 0)
                    m_inputState = MSP_CHECKSUM_V2_NATIVE;
                else
                    m_inputState = MSP_PAYLOAD_V2_NATIVE;
            }
            break;

        case MSP_PAYLOAD_V2_NATIVE:
            m_packet.payload[m_offset++] = c;
            m_crc = crc8_dvb_s2(m_crc, c);

            if (m_offset == m_packet.payloadSize) {
                m_inputState = MSP_CHECKSUM_V2_NATIVE;
            }
            break;

        case MSP_CHECKSUM_V2_NATIVE:
            if (m_crc == c) {
                m_inputState = MSP_COMMAND_RECEIVED;
            }
            else {
                m_inputState = MSP_IDLE;
            }
            break;

        default:
            m_inputState = MSP_IDLE;
            break;
    }

    if (m_inputState == MSP_COMMAND_RECEIVED) {
        return true;
    }
    return false;
}

mspPacket_t*
MSP::getReceivedPacket()
{
    return &m_packet;
}

void
MSP::markPacketReceived()
{
    m_inputState = MSP_IDLE;
}

bool
MSP::sendPacket(mspPacket_t* packet, Stream* port)
{
    if (packet->type != MSP_PACKET_COMMAND && packet->type != MSP_PACKET_RESPONSE) {
        return false;
    }
    if (packet->type == MSP_PACKET_RESPONSE && packet->payloadSize == 0) {
        return false;
    }

    port->write('$');
    port->write('X');

    if (packet->type == MSP_PACKET_COMMAND) {
        port->write('<');
    }
    else if (packet->type == MSP_PACKET_RESPONSE) {
        port->write('>');
    }

    uint8_t crc = 0;
    uint8_t headerBuffer[5];
    mspHeaderV2_t* header = (mspHeaderV2_t*)&headerBuffer[0];
    header->flags = packet->flags;
    header->function = packet->function;
    header->payloadSize = packet->payloadSize;

    for (uint8_t i = 0; i < sizeof(mspHeaderV2_t); ++i) {
        port->write(headerBuffer[i]);
        crc = crc8_dvb_s2(crc, headerBuffer[i]);
    }
    for (uint16_t i = 0; i < packet->payloadSize; ++i) {
        port->write(packet->payload[i]);
        crc = crc8_dvb_s2(crc, packet->payload[i]);
    }
    port->write(crc);

    return true;
}

uint8_t
MSP::convertToByteArray(mspPacket_t* packet, uint8_t* byteArray)
{
    uint8_t bufferPos = 0;
    if (packet->type != MSP_PACKET_COMMAND && packet->type != MSP_PACKET_RESPONSE) {
        return 0;
    }
    if (packet->type == MSP_PACKET_RESPONSE && packet->payloadSize == 0) {
        return 0;
    }

    byteArray[bufferPos++] = '$';
    byteArray[bufferPos++] = 'X';

    if (packet->type == MSP_PACKET_COMMAND) {
        byteArray[bufferPos++] = '<';
    }
    else if (packet->type == MSP_PACKET_RESPONSE) {
        byteArray[bufferPos++] = '>';
    }

    uint8_t crc = 0;
    uint8_t headerBuffer[5];
    mspHeaderV2_t* header = (mspHeaderV2_t*)&headerBuffer[0];
    header->flags = packet->flags;
    header->function = packet->function;
    header->payloadSize = packet->payloadSize;

    for (uint8_t i = 0; i < sizeof(mspHeaderV2_t); ++i) {
        byteArray[bufferPos++] = headerBuffer[i];
        crc = crc8_dvb_s2(crc, headerBuffer[i]);
    }
    for (uint16_t i = 0; i < packet->payloadSize; ++i) {
        byteArray[bufferPos++] = packet->payload[i];
        crc = crc8_dvb_s2(crc, packet->payload[i]);
    }
    byteArray[bufferPos++] = crc;

    return bufferPos;
}

uint8_t
MSP::getTotalPacketSize(mspPacket_t* packet)
{
    uint8_t totalSize = 0;
    totalSize += sizeof('$');
    totalSize += sizeof('X');

    if (packet->type == MSP_PACKET_COMMAND) {
        totalSize += sizeof('<');
    }
    else if (packet->type == MSP_PACKET_RESPONSE) {
        totalSize += sizeof('>');
    }

    totalSize += sizeof(mspHeaderV2_t);
    totalSize += packet->payloadSize;
    totalSize++; // crc

    return totalSize;
}
