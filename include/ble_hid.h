#ifndef BLESCROLL_BLE_HID_H_
#define BLESCROLL_BLE_HID_H_

#include <stdbool.h>
#include <stdint.h>

int ble_hid_init(void);
void ble_hid_scroll_enqueue(int32_t delta);
bool ble_hid_is_connected(void);
void ble_hid_diagnostic_print(void);

#endif /* BLESCROLL_BLE_HID_H_ */
