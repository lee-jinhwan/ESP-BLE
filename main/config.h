#ifndef CONFIG_H
#define CONFIG_H

/* System */
#define SCAN_TIME       1


/* Device */
#define RSSI_N          3.0
#define RSSI_TX_POWER  -55


/* IRK */
#define IRK_LIST_NUMBER 5
uint8_t irk[IRK_LIST_NUMBER][ESP_BT_OCTET16_LEN]= {
    // My iPhone
    // 4e012c1d0f76eecaf4deee3caee299d9
    {0xd9,0x99,0xe2,0xae,0x3c,0xee,0xde,0xf4,0xca,0xee,0x76,0x0f,0x1d,0x2c,0x01,0x4e}, 
    // My Apple Watch
    // 54f39ffe98bf4529f99965e49138118c
    {0x8c,0x11,0x38,0x91,0xe4,0x65,0x99,0xf9,0x29,0x45,0xbf,0x98,0xfe,0x9f,0xf3,0x54},
    // Mom's iPhone
    // cd89201334ec2e1514a129083a9d0ec6
    {0xc6,0x0e,0x9d,0x3a,0x08,0x29,0xa1,0x14,0x15,0x2e,0xec,0x34,0x13,0x20,0x89,0xcd},
    // Mom's Apple Watch
    // 65ad508bd169dbece6559a5feac8ed42
    {0x42,0xed,0xc8,0xea,0x5f,0x9a,0x55,0xe6,0xec,0xdb,0x69,0xd1,0x8b,0x50,0xad,0x65},
    // Sister's iPhone
    // 264db61825c0f58d3b103ea64d4c10da
    {0xda,0x10,0x4c,0x4d,0xa6,0x3e,0x10,0x3b,0x8d,0xf5,0xc0,0x25,0x18,0xb6,0x4d,0x26},
};

/* Door information and key */
#define DOOR_COUNT    4
const char* DOOR[] = { "D_0523_7M", "D_0523_8M", "D_0523_9M", "BLE Device" };
#define DOOR_KEY_LEN  14
uint8_t DOOR_KEY[DOOR_KEY_LEN] = {
    //    <SP>     A     C     2     0     1     0     2     1     6     0     2
    0x02, 0x20, 0x41, 0x43, 0x32, 0x30, 0x31, 0x30, 0x32, 0x31, 0x36, 0x30, 0x32, 0x0d
};

static BLEUUID SERVICE_UUID("ED2B4E3A-2820-492F-9507-DF165285E831");
static BLEUUID    CHAR_UUID("ED2B4E3B-2820-492F-9507-DF165285E831");

uint32_t RED_LED[25] = {
    0x000000, 0x000000, 0x000000, 0x000000, 0x000000,
    0x000000, 0x000000, 0x000000, 0x000000, 0x000000,
    0x000000, 0x000000, 0x000000, 0x000000, 0x000000,
    0x000000, 0x000000, 0x000000, 0x000000, 0x000000,
    0x000000, 0x000000, 0x000000, 0x000000, 0x000100,
};

uint32_t YELLOW_LED[25] = {
    0x000000, 0x000000, 0x000000, 0x000000, 0x000000,
    0x000000, 0x000000, 0x000000, 0x000000, 0x000000,
    0x000000, 0x000000, 0x000000, 0x000000, 0x000000,
    0x000000, 0x000000, 0x000000, 0x000000, 0x000000,
    0x000000, 0x000000, 0x000000, 0x000000, 0x010100,
};

uint32_t GREEN_LED[25] = {
    0x000000, 0x000000, 0x000000, 0x000000, 0x000000,
    0x000000, 0x000000, 0x000000, 0x000000, 0x000000,
    0x000000, 0x000000, 0x000000, 0x000000, 0x000000,
    0x000000, 0x000000, 0x000000, 0x000000, 0x000000,
    0x000000, 0x000000, 0x000000, 0x000000, 0x010000,
};

uint32_t BLUE_LED[25] = {
    0x000000, 0x000000, 0x000000, 0x000000, 0x000000,
    0x000000, 0x000000, 0x000000, 0x000000, 0x000000,
    0x000000, 0x000000, 0x000000, 0x000000, 0x000000,
    0x000000, 0x000000, 0x000000, 0x000000, 0x000000,
    0x000000, 0x000000, 0x000000, 0x000000, 0x000001,
};

uint32_t WHITE_LED[25] = {
    0x000000, 0x000000, 0x000000, 0x000000, 0x000000,
    0x000000, 0x000000, 0x000000, 0x000000, 0x000000,
    0x000000, 0x000000, 0x000000, 0x000000, 0x000000,
    0x000000, 0x000000, 0x000000, 0x000000, 0x000000,
    0x000000, 0x000000, 0x000000, 0x000000, 0x010101,
};

uint32_t NUMBER_TO_LED[20][25] = {
    {
        0x000000, 0x000000, 0x010101, 0x010101, 0x010101,
        0x000000, 0x000000, 0x010101, 0x000000, 0x010101,
        0x000000, 0x000000, 0x010101, 0x000000, 0x010101,
        0x000000, 0x000000, 0x010101, 0x000000, 0x010101,
        0x000000, 0x000000, 0x010101, 0x010101, 0x010101,
    },
    {
        0x000000, 0x000000, 0x000000, 0x000000, 0x010101,
        0x000000, 0x000000, 0x000000, 0x000000, 0x010101,
        0x000000, 0x000000, 0x000000, 0x000000, 0x010101,
        0x000000, 0x000000, 0x000000, 0x000000, 0x010101,
        0x000000, 0x000000, 0x000000, 0x000000, 0x010101,
    },
    {
        0x000000, 0x000000, 0x010101, 0x010101, 0x010101,
        0x000000, 0x000000, 0x000000, 0x000000, 0x010101,
        0x000000, 0x000000, 0x010101, 0x010101, 0x010101,
        0x000000, 0x000000, 0x010101, 0x000000, 0x000000,
        0x000000, 0x000000, 0x010101, 0x010101, 0x010101,
    },
    {
        0x000000, 0x000000, 0x010101, 0x010101, 0x010101,
        0x000000, 0x000000, 0x000000, 0x000000, 0x010101,
        0x000000, 0x000000, 0x010101, 0x010101, 0x010101,
        0x000000, 0x000000, 0x000000, 0x000000, 0x010101,
        0x000000, 0x000000, 0x010101, 0x010101, 0x010101,
    },
    {
        0x000000, 0x000000, 0x010101, 0x000000, 0x010101,
        0x000000, 0x000000, 0x010101, 0x000000, 0x010101,
        0x000000, 0x000000, 0x010101, 0x010101, 0x010101,
        0x000000, 0x000000, 0x000000, 0x000000, 0x010101,
        0x000000, 0x000000, 0x000000, 0x000000, 0x010101,
    },
    {
        0x000000, 0x000000, 0x010101, 0x010101, 0x010101,
        0x000000, 0x000000, 0x010101, 0x000000, 0x000000,
        0x000000, 0x000000, 0x010101, 0x010101, 0x010101,
        0x000000, 0x000000, 0x000000, 0x000000, 0x010101,
        0x000000, 0x000000, 0x010101, 0x010101, 0x010101,
    },
    {
        0x000000, 0x000000, 0x010101, 0x010101, 0x010101,
        0x000000, 0x000000, 0x010101, 0x000000, 0x000000,
        0x000000, 0x000000, 0x010101, 0x010101, 0x010101,
        0x000000, 0x000000, 0x010101, 0x000000, 0x010101,
        0x000000, 0x000000, 0x010101, 0x010101, 0x010101,
    },
    {
        0x000000, 0x000000, 0x010101, 0x010101, 0x010101,
        0x000000, 0x000000, 0x000000, 0x000000, 0x010101,
        0x000000, 0x000000, 0x000000, 0x000000, 0x010101,
        0x000000, 0x000000, 0x000000, 0x000000, 0x010101,
        0x000000, 0x000000, 0x000000, 0x000000, 0x010101,
    },
    {
        0x000000, 0x000000, 0x010101, 0x010101, 0x010101,
        0x000000, 0x000000, 0x010101, 0x000000, 0x010101,
        0x000000, 0x000000, 0x010101, 0x010101, 0x010101,
        0x000000, 0x000000, 0x010101, 0x000000, 0x010101,
        0x000000, 0x000000, 0x010101, 0x010101, 0x010101,
    },
    {
        0x000000, 0x000000, 0x010101, 0x010101, 0x010101,
        0x000000, 0x000000, 0x010101, 0x000000, 0x010101,
        0x000000, 0x000000, 0x010101, 0x010101, 0x010101,
        0x000000, 0x000000, 0x000000, 0x000000, 0x010101,
        0x000000, 0x000000, 0x010101, 0x010101, 0x010101,
    },
    {
        0x010101, 0x000000, 0x010101, 0x010101, 0x010101,
        0x010101, 0x000000, 0x010101, 0x000000, 0x010101,
        0x010101, 0x000000, 0x010101, 0x000000, 0x010101,
        0x010101, 0x000000, 0x010101, 0x000000, 0x010101,
        0x010101, 0x000000, 0x010101, 0x010101, 0x010101,
    },
    {
        0x010101, 0x000000, 0x000000, 0x000000, 0x010101,
        0x010101, 0x000000, 0x000000, 0x000000, 0x010101,
        0x010101, 0x000000, 0x000000, 0x000000, 0x010101,
        0x010101, 0x000000, 0x000000, 0x000000, 0x010101,
        0x010101, 0x000000, 0x000000, 0x000000, 0x010101,
    },
    {
        0x010101, 0x000000, 0x010101, 0x010101, 0x010101,
        0x010101, 0x000000, 0x000000, 0x000000, 0x010101,
        0x010101, 0x000000, 0x010101, 0x010101, 0x010101,
        0x010101, 0x000000, 0x010101, 0x000000, 0x000000,
        0x010101, 0x000000, 0x010101, 0x010101, 0x010101,
    },
    {
        0x010101, 0x000000, 0x010101, 0x010101, 0x010101,
        0x010101, 0x000000, 0x000000, 0x000000, 0x010101,
        0x010101, 0x000000, 0x010101, 0x010101, 0x010101,
        0x010101, 0x000000, 0x000000, 0x000000, 0x010101,
        0x010101, 0x000000, 0x010101, 0x010101, 0x010101,
    },
    {
        0x010101, 0x000000, 0x010101, 0x000000, 0x010101,
        0x010101, 0x000000, 0x010101, 0x000000, 0x010101,
        0x010101, 0x000000, 0x010101, 0x010101, 0x010101,
        0x010101, 0x000000, 0x000000, 0x000000, 0x010101,
        0x010101, 0x000000, 0x000000, 0x000000, 0x010101,
    },
    {
        0x010101, 0x000000, 0x010101, 0x010101, 0x010101,
        0x010101, 0x000000, 0x010101, 0x000000, 0x000000,
        0x010101, 0x000000, 0x010101, 0x010101, 0x010101,
        0x010101, 0x000000, 0x000000, 0x000000, 0x010101,
        0x010101, 0x000000, 0x010101, 0x010101, 0x010101,
    },
    {
        0x010101, 0x000000, 0x010101, 0x010101, 0x010101,
        0x010101, 0x000000, 0x010101, 0x000000, 0x000000,
        0x010101, 0x000000, 0x010101, 0x010101, 0x010101,
        0x010101, 0x000000, 0x010101, 0x000000, 0x010101,
        0x010101, 0x000000, 0x010101, 0x010101, 0x010101,
    },
    {
        0x010101, 0x000000, 0x010101, 0x010101, 0x010101,
        0x010101, 0x000000, 0x000000, 0x000000, 0x010101,
        0x010101, 0x000000, 0x000000, 0x000000, 0x010101,
        0x010101, 0x000000, 0x000000, 0x000000, 0x010101,
        0x010101, 0x000000, 0x000000, 0x000000, 0x010101,
    },
    {
        0x010101, 0x000000, 0x010101, 0x010101, 0x010101,
        0x010101, 0x000000, 0x010101, 0x000000, 0x010101,
        0x010101, 0x000000, 0x010101, 0x010101, 0x010101,
        0x010101, 0x000000, 0x010101, 0x000000, 0x010101,
        0x010101, 0x000000, 0x010101, 0x010101, 0x010101,
    },
    {
        0x010101, 0x000000, 0x010101, 0x010101, 0x010101,
        0x010101, 0x000000, 0x010101, 0x000000, 0x010101,
        0x010101, 0x000000, 0x010101, 0x010101, 0x010101,
        0x010101, 0x000000, 0x000000, 0x000000, 0x010101,
        0x010101, 0x000000, 0x010101, 0x010101, 0x010101,
    },
};

#endif