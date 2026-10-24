#pragma once
#if (defined(MAIN) + defined(SMOKE_TEST) + defined(SENSOR_TEST) + defined(FLASH_TEST) + defined(FLASH_DIRECTORY_TEST) + defined(CONFIG_STORE_TEST) + defined(BLUETOOTH_TEST) + defined(BLUETOOTH_NAME_TEST)) != 1
#error Select exactly one MAIN/test build flag in platformio.ini
#endif
