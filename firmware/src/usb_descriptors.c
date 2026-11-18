#include "board.h"
#include "tusb.h"

enum { ITF_CDC = 0, ITF_CDC_DATA, ITF_COUNT };
#define CONFIG_LENGTH (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN)

/* TinyUSB example VID/PID, suitable for development only. */
static const tusb_desc_device_t device_descriptor = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = 0xCAFE,
    .idProduct = 0x4011,
    .bcdDevice = 0x0200,
    .iManufacturer = 1,
    .iProduct = 2,
    .iSerialNumber = 3,
    .bNumConfigurations = 1,
};

static const uint8_t configuration_descriptor[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_COUNT, 0, CONFIG_LENGTH, 0, 100),
    TUD_CDC_DESCRIPTOR(ITF_CDC, 4, 0x81, 8, 0x02, 0x82, 64),
};

const uint8_t *tud_descriptor_device_cb(void)
{
    return (const uint8_t *)&device_descriptor;
}

const uint8_t *tud_descriptor_configuration_cb(uint8_t index)
{
    (void)index;
    return configuration_descriptor;
}

const uint16_t *tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
    (void)langid;
    static uint16_t descriptor[40];
    static const char *const strings[] = {
        "", "Bixocean", "BMI088 IMU 1000Hz", "", "Raw six-axis 1000Hz",
    };
    unsigned length = 0;
    if (index == 0u) {
        descriptor[1] = 0x0409;
        length = 1;
    } else if (index == 3u) {
        const uint32_t uid[] = {HAL_GetUIDw0(), HAL_GetUIDw1(), HAL_GetUIDw2()};
        static const char hex[] = "0123456789ABCDEF";
        for (unsigned word = 0; word < 3u; ++word) {
            for (unsigned nibble = 0; nibble < 8u; ++nibble) {
                descriptor[1u + length++] = (uint16_t)hex[
                    (uid[word] >> (28u - 4u * nibble)) & 0xFu];
            }
        }
    } else {
        if (index >= sizeof(strings) / sizeof(strings[0])) {
            return NULL;
        }
        const char *str = strings[index];
        while ((str[length] != '\0') && (length < 39u)) {
            descriptor[length + 1u] = (uint8_t)str[length];
            ++length;
        }
    }
    descriptor[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2u * length + 2u));
    return descriptor;
}
