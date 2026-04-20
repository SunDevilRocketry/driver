static const uint8_t nmea_enable_ubx_input[] = 
    "$PUBX,41,1,0003,0001,9600,0*16\r\n";

//static const uint8_t uart_baud_config[] = 
//{ 0xb5, 0x62, 0x06, 0x8a, 0x0c, 0x00, 0x00, 0x01, 0x00, 0x00, 0x01, 0x00, 0x52, 0x40, 0x00, 0x10, 0x0e, 0x00, 0x4e, 0xa9 };

static const uint8_t volt_ctrl_config[] =
{ 
    0xB5, 0x62,              /* preamble */
    0x06, 0x8A,              /* class/ID: UBX-CFG-VALSET */
    0x09, 0x00,              /* length: 9 bytes (little-endian) */
    0x00,                    /* version */
    0x01,                    /* layers: RAM */
    0x00, 0x00,              /* reserved */
    0x2E, 0x00, 0xA3, 0x10, /* key: CFG-HW-ANT_CFG_VOLTCTRL (little-endian) */
    0x01,                    /* value: true */
    0x76, 0xB2               /* CK_A, CK_B */
};

static const uint8_t ant_powerdown_disable[] =
{
    0xB5, 0x62,              /* preamble */
    0x06, 0x8A,              /* class/ID: UBX-CFG-VALSET */
    0x09, 0x00,              /* length: 9 bytes (little-endian) */
    0x00,                    /* version */
    0x01,                    /* layers: RAM */
    0x00, 0x00,              /* reserved */
    0x33, 0x00, 0xA3, 0x10, /* key: CFG-HW-ANT_CFG_PWRDOWN (little-endian) */
    0x00,                    /* value: false */
    0x7C, 0xC3               /* CK_A, CK_B */
};