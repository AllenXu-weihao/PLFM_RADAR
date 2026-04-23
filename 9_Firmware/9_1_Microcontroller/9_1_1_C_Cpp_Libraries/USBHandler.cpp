#include "USBHandler.h"
#include "diag_log.h"
#include <cstring>

USBHandler::USBHandler() {
    DIAG("USB", "USBHandler constructed, calling reset()");
    reset();
}

void USBHandler::reset() {
    DIAG("USB", "USBHandler::reset(): state -> WAITING_FOR_START");
    current_state = USBState::WAITING_FOR_START;
    start_flag_received = false;
    buffer_index = 0;
    current_settings.resetToDefaults();
}

void USBHandler::processUSBData(const uint8_t* data, uint32_t length) {
    if (data == nullptr || length == 0) {
        DIAG_WARN("USB", "processUSBData: null/empty data");
        return;
    }
    
    DIAG("USB", "processUSBData: %lu bytes, state=%d", (unsigned long)length, (int)current_state);
    
    switch (current_state) {
        case USBState::WAITING_FOR_START:
            processStartFlag(data, length);
            break;
            
        case USBState::RECEIVING_SETTINGS:
            processSettingsData(data, length);
            break;
            
        case USBState::READY_FOR_DATA:
            // Ready to receive radar data commands
            DIAG("USB", "  READY_FOR_DATA: ignoring %lu bytes", (unsigned long)length);
            break;
    }
}

void USBHandler::processStartFlag(const uint8_t* data, uint32_t length) {
    // Start flag: bytes [23, 46, 158, 237]
    const uint8_t START_FLAG[] = {23, 46, 158, 237};
    
    // Guard: need at least 4 bytes to contain a start flag.
    // Without this, length - 4 wraps to ~4 billion (uint32_t unsigned underflow)
    // and the loop reads far past the buffer boundary.
    if (length < 4) return;
    
    // Check if start flag is in the received data
    for (uint32_t i = 0; i <= length - 4; i++) {
        if (memcmp(data + i, START_FLAG, 4) == 0) {
            start_flag_received = true;
            current_state = USBState::RECEIVING_SETTINGS;
            buffer_index = 0;  // Reset buffer for settings data
            DIAG("USB", "START FLAG found at offset %lu, state -> RECEIVING_SETTINGS", (unsigned long)i);
            
            // If there's more data after the start flag, process it
            if (length > i + 4) {
                DIAG("USB", "  %lu trailing bytes after start flag, forwarding to settings parser", (unsigned long)(length - i - 4));
                processSettingsData(data + i + 4, length - i - 4);
            }
            return;
        }
    }
    DIAG("USB", "  no start flag in %lu bytes", (unsigned long)length);
}

void USBHandler::processSettingsData(const uint8_t* data, uint32_t length) {
    // Add data to buffer
    uint32_t bytes_to_copy = (length < (MAX_BUFFER_SIZE - buffer_index)) ? 
                             length : (MAX_BUFFER_SIZE - buffer_index);
    
    memcpy(usb_buffer + buffer_index, data, bytes_to_copy);
    buffer_index += bytes_to_copy;
    DIAG("USB", "  settings buffer: +%lu bytes, total=%lu/%u", (unsigned long)bytes_to_copy, (unsigned long)buffer_index, MAX_BUFFER_SIZE);
    
    // Check if we have a complete settings packet (contains "SET" and "END")
    // Minimum valid packet is "SET" + 9 doubles + 1 uint32 + "END" = 82 bytes
    // (matches RadarSettings::parseFromUSB length check).
    if (buffer_index >= 82) {
        // Look for "SET" at beginning and "END" somewhere in the packet
        bool has_set = (memcmp(usb_buffer, "SET", 3) == 0);
        bool has_end = false;
        uint32_t packet_len = 0;

        DIAG_BOOL("USB", "  packet starts with SET", has_set);

        for (uint32_t i = 3; i <= buffer_index - 3; i++) {
            if (memcmp(usb_buffer + i, "END", 3) == 0) {
                has_end = true;
                packet_len = i + 3;
                DIAG("USB", "  END marker found at offset %lu, packet_len=%lu", (unsigned long)i, (unsigned long)packet_len);

                // Parse the complete packet up to "END"
                if (has_set && current_settings.parseFromUSB(usb_buffer, packet_len)) {
                    current_state = USBState::READY_FOR_DATA;
                    DIAG("USB", "  Settings parsed OK, state -> READY_FOR_DATA");
                } else {
                    DIAG_ERR("USB", "  Settings parse FAILED (has_set=%d)", has_set);
                }
                break;
            }
        }

        // [MCU-N9 FIX] Drain the consumed packet bytes (or false-positive END)
        // so a parse failure doesn't leave the buffer stuck on the same bytes
        // until MAX_BUFFER_SIZE overflow. Slide any remaining bytes left.
        if (has_end && packet_len > 0) {
            uint32_t remaining = buffer_index - packet_len;
            if (remaining > 0) {
                memmove(usb_buffer, usb_buffer + packet_len, remaining);
            }
            buffer_index = remaining;
        } else if (buffer_index >= MAX_BUFFER_SIZE) {
            DIAG_WARN("USB", "  Buffer full (%u) without END marker -- resetting", MAX_BUFFER_SIZE);
            buffer_index = 0;
        }
    }
}