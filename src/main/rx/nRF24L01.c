#include <stdbool.h>
#include <stdint.h>

#include <platform.h>
#include "build/build_config.h"
#include "drivers/system.h"
#include "fc/runtime_config.h"

#include "rx/rx.h"

#include "platform.h"

#include "drivers/sensor.h"

#include "drivers/bus_spi_soft.h"

#include "fc/rc_controls.h"
#include "sensors/battery.h"
#include "io/gps.h"

#include "rx/rx.h"
#include "rx/nRF24L01.h"

//

#include "common/maths.h"
#include "common/axis.h"
#include "common/color.h"
#include "common/utils.h"

#include "drivers/gpio.h"
#include "drivers/serial.h"
#include "drivers/compass.h"
#include "drivers/timer.h"
#include "drivers/pwm_rx.h"
#include "drivers/accgyro.h"
#include "drivers/light_led.h"

#include "sensors/sensors.h"
#include "sensors/boardalignment.h"
#include "sensors/rangefinder.h"
#include "sensors/compass.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/gyro.h"

#include "io/beeper.h"
#include "io/display.h"
#include "io/escservo.h"
#include "io/gimbal.h"
#include "io/gps.h"
#include "io/ledstrip.h"
#include "io/serial.h"
#include "io/serial_cli.h"
#include "io/serial_msp.h"
#include "io/statusindicator.h"

#include "rx/msp.h"

#include "telemetry/telemetry.h"

#include "flight/mixer.h"
#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/navigation_rewrite.h"

#include "config/config.h"
#include "config/config_profile.h"
#include "config/config_master.h"
//

#ifdef NRF24_ARD

/***************************** Configuration ********************************/

// nRF24L01 pin setup for Naze32 using soft SPI

// GND  --> any ground pin
// VCC  --> 3v supply
// CSN  --> receiver pin 5
// SCK  --> receiver pin 6
// MISO --> receiver pin 7
// MOSI --> receiver pin 8
// CE   --> receiver pin 3

#define NRF24_CSN_PORT  GPIOA
#define NRF24_CSN_PIN   GPIO_Pin_6

#define NRF24_SCK_PORT  GPIOA
#define NRF24_SCK_PIN   GPIO_Pin_7

#define NRF24_MISO_PORT GPIOB
#define NRF24_MISO_PIN  GPIO_Pin_0

#define NRF24_MOSI_PORT GPIOB
#define NRF24_MOSI_PIN  GPIO_Pin_1

#define NRF24_CE_PORT   GPIOA
#define NRF24_CE_PIN    GPIO_Pin_1

//Control array index
#define INDEX_THROTTLE  0
#define INDEX_YAW       1
#define INDEX_PITCH     2
#define INDEX_ROLL      3

#define INDEX_AUX1      0
#define INDEX_AUX2      1
#define INDEX_AUX3      2
#define INDEX_AUX4      3
#define INDEX_AUX5      4
#define INDEX_AUX6      5

#define NRF24_CHANNEL_COUNT 10

static int16_t nrfChannelData[NRF24_CHANNEL_COUNT]; //to callback
static uint16_t nrfReadRawRC(rxRuntimeConfig_t *rxRuntimeConfig, uint8_t chan);

typedef struct nrf24Payload {
    int16_t control_values[4];
    int16_t aux_values[6];
} nrf24Payload;

nrf24Payload payload;

typedef struct nrf24Telem {
    uint16_t vbat;
    int32_t latitude;
    int32_t longitude;
} nrf24Telem;

nrf24Telem telem;


#define NRF24_FAILSAFE_TIME_MS  1300

#define CONTROL_FRAME_SIZE      20
#define TELEM_FRAME_SIZE        10

#define TELEM_SEND_DELAY        2000

void resetPayload()
{
    payload.control_values[INDEX_THROTTLE] = 885;
    payload.control_values[INDEX_YAW] = 1500;
    payload.control_values[INDEX_PITCH] = 1500;
    payload.control_values[INDEX_ROLL] = 1500;
}

/****************************************************************************/

// *** All settings here must match the transmitter! ***
void startAsPrimaryReceiver()
{
    //nrf24_setChannel(NRF_CHANNEL);
    nrf24_setChannel(masterConfig.nrfChannel);
    nrf24_setDataRate(RF24_250KBPS);
    
    nrf24_setAutoAck(false);
    nrf24_setPayloadSize(CONTROL_FRAME_SIZE);
    
    //nrf24_openReadingPipe(1, CONTROL_PIPE);
    //nrf24_openWritingPipe(TELEM_PIPE);
    nrf24_openReadingPipe(1, masterConfig.nrfControlPipe);
    nrf24_openWritingPipe(masterConfig.nrfTelemetryPipe);
    
    nrf24_startListening();
}

/****************************************************************************/

void setRcDataFromPayload() {
    nrfChannelData[0] = payload.control_values[INDEX_ROLL];
    nrfChannelData[1] = payload.control_values[INDEX_PITCH];
    nrfChannelData[2] = payload.control_values[INDEX_YAW];
    nrfChannelData[3] = payload.control_values[INDEX_THROTTLE];
    
    nrfChannelData[4] = payload.aux_values[INDEX_AUX1];
    nrfChannelData[5] = payload.aux_values[INDEX_AUX2];
    nrfChannelData[6] = payload.aux_values[INDEX_AUX3];
    nrfChannelData[7] = payload.aux_values[INDEX_AUX4];
    nrfChannelData[8] = payload.aux_values[INDEX_AUX5];
    nrfChannelData[9] = payload.aux_values[INDEX_AUX6];
}

/****************************************************************************/

struct SoftSPIDevice nrf24SoftSPIDevice;

// Alter this function if you want to use hardware SPI.
uint8_t nrf24_readwrite(uint8_t data) {
    
    return softSpiTransferByte(&nrf24SoftSPIDevice, data);
    
}

/****************************************************************************/







// End of configuration.  Implementation follows....







/****************************************************************************/

// Based on: https://github.com/gcopeland/RF24

bool wide_band;                 // 2Mbs data rate in use?
bool p_variant;                 // False for RF24L01 and true for RF24L01P
uint8_t payload_size;           // Fixed size of payloads
bool ack_payload_available;     // Whether there is an ack payload waiting
bool dynamic_payloads_enabled;  // Whether dynamic payloads are enabled.
uint8_t ack_payload_length;     // Dynamic size of pending ack payload.
uint64_t pipe0_reading_address; // Last address set on pipe 0 for reading.
uint32_t lastRecvTime;
uint32_t telemetryLastSendTime; //Telemetry last send time

#define CSN_L SOFTSPI_CSN_L(nrf24SoftSPIDevice)
#define CSN_H SOFTSPI_CSN_H(nrf24SoftSPIDevice)

#define SCK_L SOFTSPI_SCK_L(nrf24SoftSPIDevice)
#define SCK_H SOFTSPI_SCK_H(nrf24SoftSPIDevice)

#define MOSI_L SOFTSPI_MOSI_L(nrf24SoftSPIDevice)
#define MOSI_H SOFTSPI_MOSI_H(nrf24SoftSPIDevice)

#define MISO_R SOFTSPI_MISO_R(nrf24SoftSPIDevice)

#define CE_L SOFTSPI_CE_L(nrf24SoftSPIDevice)
#define CE_H SOFTSPI_CE_H(nrf24SoftSPIDevice)

#define MIN_(a, b) ((a) < (b) ? (a) : (b))
#define MAX_(a, b) ((a) > (b) ? (a) : (b))

#define _BV(x) (1<<(x))

// nRF24L01 registers
#define NRF24_CONFIG        0x00
#define NRF24_EN_AA         0x01
#define NRF24_EN_RXADDR     0x02
#define NRF24_SETUP_AW      0x03
#define NRF24_SETUP_RETR    0x04
#define NRF24_RF_CH         0x05
#define NRF24_RF_SETUP      0x06
#define NRF24_STATUS        0x07
#define NRF24_OBSERVE_TX    0x08
#define NRF24_CD            0x09
#define NRF24_RX_ADDR_P0    0x0A
#define NRF24_RX_ADDR_P1    0x0B
#define NRF24_RX_ADDR_P2    0x0C
#define NRF24_RX_ADDR_P3    0x0D
#define NRF24_RX_ADDR_P4    0x0E
#define NRF24_RX_ADDR_P5    0x0F
#define NRF24_TX_ADDR       0x10
#define NRF24_RX_PW_P0      0x11
#define NRF24_RX_PW_P1      0x12
#define NRF24_RX_PW_P2      0x13
#define NRF24_RX_PW_P3      0x14
#define NRF24_RX_PW_P4      0x15
#define NRF24_RX_PW_P5      0x16
#define NRF24_FIFO_STATUS   0x17
#define NRF24_DYNPD         0x1C
#define NRF24_FEATURE	    0x1D

// nRF24L01 bit mnemonics
#define NRF24_MASK_RX_DR    6
#define NRF24_MASK_TX_DS    5
#define NRF24_MASK_MAX_RT   4
#define NRF24_EN_CRC        3
#define NRF24_CRCO          2
#define NRF24_PWR_UP        1
#define NRF24_PRIM_RX       0
#define NRF24_ENAA_P5       5
#define NRF24_ENAA_P4       4
#define NRF24_ENAA_P3       3
#define NRF24_ENAA_P2       2
#define NRF24_ENAA_P1       1
#define NRF24_ENAA_P0       0
#define NRF24_ERX_P5        5
#define NRF24_ERX_P4        4
#define NRF24_ERX_P3        3
#define NRF24_ERX_P2        2
#define NRF24_ERX_P1        1
#define NRF24_ERX_P0        0
#define NRF24_AW            0
#define NRF24_ARD_REG       4
#define NRF24_ARC           0
#define NRF24_PLL_LOCK      4
#define NRF24_RF_DR         3
#define NRF24_RF_PWR        6
#define NRF24_RX_DR         6
#define NRF24_TX_DS         5
#define NRF24_MAX_RT        4
#define NRF24_RX_P_NO       1
#define NRF24_TX_FULL       0
#define NRF24_PLOS_CNT      4
#define NRF24_ARC_CNT       0
#define NRF24_TX_REUSE      6
#define NRF24_FIFO_FULL     5
#define NRF24_TX_EMPTY      4
#define NRF24_RX_FULL       1
#define NRF24_RX_EMPTY      0
#define NRF24_DPL_P5	    5
#define NRF24_DPL_P4	    4
#define NRF24_DPL_P3	    3
#define NRF24_DPL_P2	    2
#define NRF24_DPL_P1	    1
#define NRF24_DPL_P0	    0
#define NRF24_EN_DPL	    2
#define NRF24_EN_ACK_PAY    1
#define NRF24_EN_DYN_ACK    0

// nRF24L01 instruction mnemonics
#define NRF24_R_REGISTER    0x00
#define NRF24_W_REGISTER    0x20
#define NRF24_REGISTER_MASK 0x1F
#define NRF24_ACTIVATE      0x50
#define NRF24_R_RX_PL_WID   0x60
#define NRF24_R_RX_PAYLOAD  0x61
#define NRF24_W_TX_PAYLOAD  0xA0
#define NRF24_W_ACK_PAYLOAD 0xA8
#define NRF24_FLUSH_TX      0xE1
#define NRF24_FLUSH_RX      0xE2
#define NRF24_REUSE_TX_PL   0xE3
#define NRF24_NOP           0xFF

// Non-P omissions
#define NRF24_LNA_HCURR     0

// P model memory Map
#define NRF24_RPD           0x09

// P model bit Mnemonics
#define NRF24_RF_DR_LOW     5
#define NRF24_RF_DR_HIGH    3
#define NRF24_RF_PWR_LOW    1
#define NRF24_RF_PWR_HIGH   2



/****************************************************************************/

uint8_t nrf24_read_buffer(uint8_t reg, uint8_t* buf, uint8_t len)
{
    uint8_t status,i;
    
    CSN_L;
    status = nrf24_readwrite(NRF24_R_REGISTER | ( NRF24_REGISTER_MASK & reg ));
    for (i = 0; i < len; i++) buf[i] = nrf24_readwrite(0);
    CSN_H;
    
    return status;
}

/****************************************************************************/

uint8_t nrf24_read_register(uint8_t reg)
{
    CSN_L;
    nrf24_readwrite(NRF24_R_REGISTER | ( NRF24_REGISTER_MASK & reg ));
    uint8_t result = nrf24_readwrite(0x00);
    CSN_H;
    
    return result;
}

/****************************************************************************/

uint8_t nrf24_write_buffer(uint8_t reg, const uint8_t* buf, uint8_t len)
{
    uint8_t status,i;
    
    CSN_L;
    status = nrf24_readwrite(NRF24_W_REGISTER | ( NRF24_REGISTER_MASK & reg ));
    for (i = 0; i < len; i++) nrf24_readwrite(*buf++);
    CSN_H;
    
    return status;
}

/****************************************************************************/

uint8_t nrf24_write_register(uint8_t reg, uint8_t value)
{
    uint8_t status;
    
    CSN_L;
    status = nrf24_readwrite(NRF24_W_REGISTER | ( NRF24_REGISTER_MASK & reg ));
    nrf24_readwrite(value);
    CSN_H;
    
    return status;
}

/****************************************************************************/

uint8_t nrf24_write_payload(const uint8_t* buf, uint8_t len)
{
    uint8_t status;
    
    const uint8_t* current = buf;
    
    uint8_t data_len = MIN_(len,payload_size);
    uint8_t blank_len = dynamic_payloads_enabled ? 0 : payload_size - data_len;
    
    CSN_L;
    status = nrf24_readwrite( NRF24_W_TX_PAYLOAD );
    while ( data_len-- )
        nrf24_readwrite(*current++);
    while ( blank_len-- )
        nrf24_readwrite(0);
    CSN_H;
    
    return status;
}

/****************************************************************************/

uint8_t nrf24_read_payload(uint8_t* buf, uint8_t len)
{
    uint8_t status;
    uint8_t* current = buf;
    
    uint8_t data_len = MIN_(len,payload_size);
    uint8_t blank_len = dynamic_payloads_enabled ? 0 : payload_size - data_len;
    
    CSN_L;
    status = nrf24_readwrite( NRF24_R_RX_PAYLOAD );
    while ( data_len-- )
        *current++ = nrf24_readwrite(0xff);
    while ( blank_len-- )
        nrf24_readwrite(0xff);
    CSN_H;
    
    return status;
}

/****************************************************************************/

uint8_t nrf24_flush_rx(void)
{
    uint8_t status;
    
    CSN_L;
    status = nrf24_readwrite( NRF24_FLUSH_RX );
    CSN_H;
    
    return status;
}

/****************************************************************************/

uint8_t nrf24_flush_tx(void)
{
    uint8_t status;
    
    CSN_L;
    status = nrf24_readwrite( NRF24_FLUSH_TX );
    CSN_H;
    
    return status;
}

/****************************************************************************/

uint8_t nrf24_get_status(void)
{
    uint8_t status = 0;
    
    CSN_L;
    status = nrf24_readwrite( NRF24_NOP );
    CSN_H;
    
    return status;
}

/****************************************************************************/

void rxNRF24InitArd(rxRuntimeConfig_t *rxRuntimeConfig, rcReadRawDataPtr *callback) {
    
    rxRuntimeConfig->channelCount = NRF24_CHANNEL_COUNT;
    
    if (callback)
        *callback = nrfReadRawRC;
    
    nrf24SoftSPIDevice.csn_port = GPIOA;
    nrf24SoftSPIDevice.csn_pin = GPIO_Pin_6;
    
    nrf24SoftSPIDevice.sck_port = GPIOA;
    nrf24SoftSPIDevice.sck_pin = GPIO_Pin_7;
    
    nrf24SoftSPIDevice.miso_port = GPIOB;
    nrf24SoftSPIDevice.miso_pin = GPIO_Pin_0;
    
    nrf24SoftSPIDevice.mosi_port = GPIOB;
    nrf24SoftSPIDevice.mosi_pin = GPIO_Pin_1;
    
    nrf24SoftSPIDevice.ce_port = GPIOA;
    nrf24SoftSPIDevice.ce_pin = GPIO_Pin_1;
    
    softSpiInit(&nrf24SoftSPIDevice);
    
    CSN_H;
    
    wide_band = true;
    p_variant = false;
    payload_size = 32;
    ack_payload_available = false;
    dynamic_payloads_enabled = false;
    pipe0_reading_address = 0;
    lastRecvTime = 0;
    telemetryLastSendTime = 0;
    
    // Must allow the radio time to settle else configuration bits will not necessarily stick.
    // This is actually only required following power up but some settling time also appears to
    // be required after resets too. For full coverage, we'll always assume the worst.
    // Enabling 16b CRC is by far the most obvious case if the wrong timing is used - or skipped.
    // Technically we require 4.5ms + 14us as a worst case. We'll just call it 5ms for good measure.
    // WARNING: Delay is based on P-variant whereby non-P *may* require different timing.
    delay( 5 ) ;
    
    // Set 1500uS (minimum for 32B payload in ESB@250KBPS) timeouts, to make testing a little easier
    // WARNING: If this is ever lowered, either 250KBS mode with AA is broken or maximum packet
    // sizes must never be used. See documentation for a more complete explanation.
    //write_register(SETUP_RETR,(0B0100 << ARD) | (0B1111 << ARC));
    nrf24_write_register(NRF24_SETUP_RETR,(4 << NRF24_ARD_REG) | (15 << NRF24_ARC));
    
    // Restore our default PA level
    nrf24_setPALevel( RF24_PA_MAX ) ;
    
    // Determine if this is a p or non-p RF24 module and then
    // reset our data rate back to default value. This works
    // because a non-P variant won't allow the data rate to
    // be set to 250Kbps.
    if( nrf24_setDataRate( RF24_250KBPS ) )
    {
        p_variant = true ;
    }
    
    // Then set the data rate to the slowest (and most reliable) speed supported by all
    // hardware.
    
    //nrf24_setDataRate( RF24_1MBPS ); //edit
    
    // Initialize CRC and request 2-byte (16bit) CRC
    nrf24_setCRCLength( RF24_CRC_16 ) ;
    
    // Disable dynamic payloads, to match dynamic_payloads_enabled setting
    nrf24_write_register(NRF24_DYNPD,0);
    
    // Reset current status
    // Notice reset and flush is the last thing we do
    nrf24_write_register(NRF24_STATUS,_BV(NRF24_RX_DR) | _BV(NRF24_TX_DS) | _BV(NRF24_MAX_RT) );
    
    // Set up default configuration.  Callers can always change it later.
    // This channel should be universally safe and not bleed over into adjacent
    // spectrum.
    
    nrf24_setChannel(masterConfig.nrfChannel);
    
    // Flush buffers
    nrf24_flush_rx();
    nrf24_flush_tx();
    
    resetPayload();
    startAsPrimaryReceiver();
}

static uint16_t nrfReadRawRC(rxRuntimeConfig_t *rxRuntimeConfig, uint8_t chan)
{
    UNUSED(rxRuntimeConfig);
    return nrfChannelData[chan];
}

/****************************************************************************/

bool rxNRF24ReceivePacketArd()
{
    bool receivedPacket = false;
    
    if (nrf24_available()) {
        nrf24_read(&payload, CONTROL_FRAME_SIZE);
        receivedPacket = true;
    }
    
    uint32_t now = millis();
    
    if (now - telemetryLastSendTime > TELEM_SEND_DELAY) {
        telem.vbat = vbat;
        
        /*
        if (STATE(GPS_FIX)) {
            telem.latitude = gpsSol.llh.lat;
            telem.longitude = gpsSol.llh.lon;
        } else {
            telem.latitude = 0;
            telem.longitude = 0;
        }
        */
        
        nrf24_stopListening();
        nrf24_startWrite(&telem, TELEM_FRAME_SIZE);
        delay(2);
        nrf24_startListening();
        
        telemetryLastSendTime = now;
    }
    
    if (receivedPacket) {
        lastRecvTime = now;
    }
    else if (now - lastRecvTime > NRF24_FAILSAFE_TIME_MS) {
        //signal lost
        return false;
    }
    
    setRcDataFromPayload();
    
    return receivedPacket;
}

/****************************************************************************/

bool nrf24_setChannel(uint8_t channel)
{
    // TODO: This method could take advantage of the 'wide_band' calculation
    // done in setChannel() to require certain channel spacing.
    
    const uint8_t max_channel = 127;
    uint8_t val = MIN_(channel,max_channel);
    nrf24_write_register(NRF24_RF_CH,val);
    return nrf24_read_register(NRF24_RF_CH) == val;
}

/****************************************************************************/

void nrf24_setPayloadSize(uint8_t size)
{
    const uint8_t max_payload_size = 32;
    payload_size = MIN_(size,max_payload_size);
}

/****************************************************************************/

uint8_t nrf24_getPayloadSize(void)
{
    return payload_size;
}

/****************************************************************************/

void nrf24_startListening(void)
{
    nrf24_write_register(NRF24_CONFIG, nrf24_read_register(NRF24_CONFIG) | _BV(NRF24_PWR_UP) | _BV(NRF24_PRIM_RX));
    nrf24_write_register(NRF24_STATUS, _BV(NRF24_RX_DR) | _BV(NRF24_TX_DS) | _BV(NRF24_MAX_RT) );
    
    // Restore the pipe0 adddress, if exists
    if (pipe0_reading_address)
        nrf24_write_buffer(NRF24_RX_ADDR_P0, (const uint8_t*)(&pipe0_reading_address), 5);
    
    // Flush buffers
    nrf24_flush_rx();
    nrf24_flush_tx();
    
    // Go!
    CE_H;
    
    // wait for the radio to come up (130us actually only needed)
    delayMicroseconds(130);
}

/****************************************************************************/

void nrf24_stopListening(void)
{
    CE_L;
    GPIO_ResetBits(GPIOA, GPIO_Pin_1);
    nrf24_flush_tx();
    nrf24_flush_rx();
}

/****************************************************************************/

void nrf24_powerDown(void)
{
    nrf24_write_register(NRF24_CONFIG,nrf24_read_register(NRF24_CONFIG) & ~_BV(NRF24_PWR_UP));
}

/****************************************************************************/

void nrf24_powerUp(void)
{
    nrf24_write_register(NRF24_CONFIG,nrf24_read_register(NRF24_CONFIG) | _BV(NRF24_PWR_UP));
}

/******************************************************************/

bool nrf24_write( const void* buf, uint8_t len )
{
    bool result = false;
    
    // Begin the write
    nrf24_startWrite(buf,len);
    
    // ------------
    // At this point we could return from a non-blocking write, and then call
    // the rest after an interrupt
    
    // Instead, we are going to block here until we get TX_DS (transmission completed and ack'd)
    // or MAX_RT (maximum retries, transmission failed).  Also, we'll timeout in case the radio
    // is flaky and we get neither.
    
    // IN the end, the send should be blocking.  It comes back in 60ms worst case, or much faster
    // if I tighted up the retry logic.  (Default settings will be 1500us.
    // Monitor the send
    uint8_t observe_tx;
    uint8_t status;
    uint32_t sent_at = millis();
    const uint32_t timeout = 500; //ms to wait for timeout
    do
    {
        status = nrf24_read_buffer(NRF24_OBSERVE_TX,&observe_tx,1);
        //IF_SERIAL_DEBUG(Serial.print(observe_tx,HEX));
    }
    while( ! ( status & ( _BV(NRF24_TX_DS) | _BV(NRF24_MAX_RT) ) ) && ( millis() - sent_at < timeout ) );
    
    // The part above is what you could recreate with your own interrupt handler,
    // and then call this when you got an interrupt
    // ------------
    
    // Call this when you get an interrupt
    // The status tells us three things
    // * The send was successful (TX_DS)
    // * The send failed, too many retries (MAX_RT)
    // * There is an ack packet waiting (RX_DR)
    bool tx_ok, tx_fail;
    nrf24_whatHappened(&tx_ok,&tx_fail,&ack_payload_available);
    
    //printf("%u%u%u\r\n",tx_ok,tx_fail,ack_payload_available);
    
    result = tx_ok;
    //IF_SERIAL_DEBUG(Serial.print(result?"...OK.":"...Failed"));
    
    // Handle the ack packet
    if ( ack_payload_available )
    {
        ack_payload_length = nrf24_getDynamicPayloadSize();
        //IF_SERIAL_DEBUG(Serial.print("[AckPacket]/"));
        //IF_SERIAL_DEBUG(Serial.println(ack_payload_length,DEC));
    }
    
    // Yay, we are done.
    
    // Power down
    nrf24_powerDown();
    
    // Flush buffers (Is this a relic of past experimentation, and not needed anymore??)
    nrf24_flush_tx();
    
    return result;
}
/****************************************************************************/

void nrf24_startWrite( const void* buf, uint8_t len )
{
    // Transmitter power-up
    nrf24_write_register(NRF24_CONFIG, ( nrf24_read_register(NRF24_CONFIG) | _BV(NRF24_PWR_UP) ) & ~_BV(NRF24_PRIM_RX) );
    delayMicroseconds(150);
    
    // Send the payload
    nrf24_write_payload( buf, len );
    
    // Allons!
    CE_H;
    GPIO_SetBits(GPIOA, GPIO_Pin_1);
    delayMicroseconds(15);
    CE_L;
    GPIO_ResetBits(GPIOA, GPIO_Pin_1);
}

/****************************************************************************/

uint8_t nrf24_getDynamicPayloadSize(void)
{
    uint8_t result = 0;
    
    CSN_L;
    nrf24_readwrite( NRF24_R_RX_PL_WID );
    result = nrf24_readwrite(0xff);
    CSN_H;
    
    return result;
}

/****************************************************************************/

// Read nRF24L01 register
// input:
//   reg - register number
// output: register value
uint8_t nRF24_ReadReg(uint8_t reg) {
    uint8_t value;
    
    CSN_L;
    nrf24_readwrite(reg);
    value = nrf24_readwrite(0x00);
    CSN_H;
    
    return value;
}

/****************************************************************************/

bool nrf24_available(void)
{
    return nrf24_available_perPipe(0);
}

/****************************************************************************/

/*uint8_t nRF24_DataReady(void) {
 uint8_t status;
 status = nRF24_ReadReg(NRF24_STATUS);
 if (status & NRF24_MASK_RX_DR) return 1;
 // Checking RX_DR isn't good enough, there's can be some data in FIFO
 status = nRF24_ReadReg(NRF24_FIFO_STATUS);
 return (status & NRF24_RX_EMPTY) ? 0 : 1;
 }*/

/****************************************************************************/

bool nrf24_available_perPipe(uint8_t* pipe_num)
{
    uint8_t status = nrf24_get_status();
    
    // Too noisy, enable if you really want lots o data!!
    //IF_SERIAL_DEBUG(print_status(status));
    
    bool result = ( status & _BV(NRF24_RX_DR) );
    
    if (result)
    {
        // If the caller wants the pipe number, include that
        if ( pipe_num )
            //*pipe_num = ( status >> RX_P_NO ) & B111;
            *pipe_num = ( status >> NRF24_RX_P_NO ) & 7;
        
        // Clear the status bit
        
        // ??? Should this REALLY be cleared now?  Or wait until we
        // actually READ the payload?
        
        nrf24_write_register(NRF24_STATUS,_BV(NRF24_RX_DR) );
        
        // Handle ack payload receipt
        if ( status & _BV(NRF24_TX_DS) )
        {
            nrf24_write_register(NRF24_STATUS,_BV(NRF24_TX_DS));
        }
    }
    
    return result;
}

/****************************************************************************/

bool nrf24_read( void* buf, uint8_t len )
{
    // Fetch the payload
    nrf24_read_payload( buf, len );
    
    // was this the last of the data available?
    return nrf24_read_register(NRF24_FIFO_STATUS) & _BV(NRF24_RX_EMPTY);
}

/****************************************************************************/

void nrf24_whatHappened(bool* tx_ok,bool* tx_fail,bool* rx_ready)
{
    // Read the status & reset the status in one easy call
    // Or is that such a good idea?
    uint8_t status = nrf24_write_register(NRF24_STATUS,_BV(NRF24_RX_DR) | _BV(NRF24_TX_DS) | _BV(NRF24_MAX_RT) );
    
    // Report to the user what happened
    *tx_ok = status & _BV(NRF24_TX_DS);
    *tx_fail = status & _BV(NRF24_MAX_RT);
    *rx_ready = status & _BV(NRF24_RX_DR);
}

/****************************************************************************/

void nrf24_openWritingPipe(uint64_t value)
{
    // Note that AVR 8-bit uC's store this LSB first, and the NRF24L01(+)
    // expects it LSB first too, so we're good.
    
    nrf24_write_buffer(NRF24_RX_ADDR_P0, (uint8_t*)(&value), 5);
    nrf24_write_buffer(NRF24_TX_ADDR, (uint8_t*)(&value), 5);
    
    const uint8_t max_payload_size = 32;
    nrf24_write_register(NRF24_RX_PW_P0,MIN_(payload_size,max_payload_size));
}

/****************************************************************************/

static const uint8_t child_pipe[] =
{
    NRF24_RX_ADDR_P0, NRF24_RX_ADDR_P1, NRF24_RX_ADDR_P2, NRF24_RX_ADDR_P3, NRF24_RX_ADDR_P4, NRF24_RX_ADDR_P5
};
static const uint8_t child_payload_size[] =
{
    NRF24_RX_PW_P0, NRF24_RX_PW_P1, NRF24_RX_PW_P2, NRF24_RX_PW_P3, NRF24_RX_PW_P4, NRF24_RX_PW_P5
};
static const uint8_t child_pipe_enable[] =
{
    NRF24_ERX_P0, NRF24_ERX_P1, NRF24_ERX_P2, NRF24_ERX_P3, NRF24_ERX_P4, NRF24_ERX_P5
};

void nrf24_openReadingPipe(uint8_t child, uint64_t address)
{
    // If this is pipe 0, cache the address.  This is needed because
    // openWritingPipe() will overwrite the pipe 0 address, so
    // startListening() will have to restore it.
    if (child == 0)
        pipe0_reading_address = address;
    
    if (child <= 6)
    {
        // For pipes 2-5, only write the LSB
        if ( child < 2 )
            nrf24_write_buffer(child_pipe[child], (const uint8_t*)(&address), 5);
        else
            nrf24_write_buffer(child_pipe[child], (const uint8_t*)(&address), 1);
        
        nrf24_write_register(child_payload_size[child],payload_size);
        
        // Note it would be more efficient to set all of the bits for all open
        // pipes at once.  However, I thought it would make the calling code
        // more simple to do it this way.
        nrf24_write_register(NRF24_EN_RXADDR,nrf24_read_register(NRF24_EN_RXADDR) | _BV(child_pipe_enable[child]));
    }
}

/****************************************************************************/

void nrf24_toggle_features(void)
{
    CSN_L;
    nrf24_readwrite( NRF24_ACTIVATE );
    nrf24_readwrite( 0x73 );
    CSN_H;
}

/****************************************************************************/

void nrf24_enableDynamicPayloads(void)
{
    // Enable dynamic payload throughout the system
    nrf24_write_register(NRF24_FEATURE,nrf24_read_register(NRF24_FEATURE) | _BV(NRF24_EN_DPL) );
    
    // If it didn't work, the features are not enabled
    if ( ! nrf24_read_register(NRF24_FEATURE) )
    {
        // So enable them and try again
        nrf24_toggle_features();
        nrf24_write_register(NRF24_FEATURE,nrf24_read_register(NRF24_FEATURE) | _BV(NRF24_EN_DPL) );
    }
    
    //IF_SERIAL_DEBUG(printf("FEATURE=%i\r\n",read_register(FEATURE)));
    
    // Enable dynamic payload on all pipes
    //
    // Not sure the use case of only having dynamic payload on certain
    // pipes, so the library does not support it.
    nrf24_write_register(NRF24_DYNPD,nrf24_read_register(NRF24_DYNPD) | _BV(NRF24_DPL_P5) | _BV(NRF24_DPL_P4) | _BV(NRF24_DPL_P3) | _BV(NRF24_DPL_P2) | _BV(NRF24_DPL_P1) | _BV(NRF24_DPL_P0));
    
    dynamic_payloads_enabled = true;
}

/****************************************************************************/

void nrf24_enableAckPayload(void)
{
    //
    // enable ack payload and dynamic payload features
    //
    
    nrf24_write_register(NRF24_FEATURE,nrf24_read_register(NRF24_FEATURE) | _BV(NRF24_EN_ACK_PAY) | _BV(NRF24_EN_DPL) );
    
    // If it didn't work, the features are not enabled
    if ( ! nrf24_read_register(NRF24_FEATURE) )
    {
        // So enable them and try again
        nrf24_toggle_features();
        nrf24_write_register(NRF24_FEATURE,nrf24_read_register(NRF24_FEATURE) | _BV(NRF24_EN_ACK_PAY) | _BV(NRF24_EN_DPL) );
    }
    
    //IF_SERIAL_DEBUG(printf("FEATURE=%i\r\n",read_register(FEATURE)));
    
    //
    // Enable dynamic payload on pipes 0 & 1
    //
    
    nrf24_write_register(NRF24_DYNPD,nrf24_read_register(NRF24_DYNPD) | _BV(NRF24_DPL_P1) | _BV(NRF24_DPL_P0));
}

/****************************************************************************/

void nrf24_writeAckPayload(uint8_t pipe, const void* buf, uint8_t len)
{
    const uint8_t* current = (const uint8_t*)(buf);
    
    CSN_L;
    //nrf24_readwrite( W_ACK_PAYLOAD | ( pipe & B111 ) );
    nrf24_readwrite( NRF24_W_ACK_PAYLOAD | ( pipe & 7 ) );
    const uint8_t max_payload_size = 32;
    uint8_t data_len = MIN_(len,max_payload_size);
    while ( data_len-- )
        nrf24_readwrite(*current++);
    
    CSN_H;
}

/****************************************************************************/

bool nrf24_isAckPayloadAvailable(void)
{
    bool result = ack_payload_available;
    ack_payload_available = false;
    return result;
}

/****************************************************************************/

bool nrf24_isPVariant(void)
{
    return p_variant ;
}

/****************************************************************************/

bool nrf24_setAutoAck(bool enable)
{
    uint8_t val = 0;
    if ( enable )
        val = 63; // B111111
    
    nrf24_write_register(NRF24_EN_AA, val);
    
    return nrf24_read_register(NRF24_EN_AA) == val;
}

/****************************************************************************/

void nrf24_setAutoAck_perPipe( uint8_t pipe, bool enable )
{
    if ( pipe <= 6 )
    {
        uint8_t en_aa = nrf24_read_register( NRF24_EN_AA ) ;
        if( enable )
        {
            en_aa |= _BV(pipe) ;
        }
        else
        {
            en_aa &= ~_BV(pipe) ;
        }
        nrf24_write_register( NRF24_EN_AA, en_aa ) ;
    }
}

/****************************************************************************/

bool nrf24_testCarrier(void)
{
    return ( nrf24_read_register(NRF24_CD) & 1 );
}

/****************************************************************************/

bool nrf24_testRPD(void)
{
    return ( nrf24_read_register(NRF24_RPD) & 1 ) ;
}

/****************************************************************************/

void nrf24_setPALevel(nrf24_pa_dbm_e level)
{
    uint8_t setup = nrf24_read_register(NRF24_RF_SETUP) ;
    setup &= ~(_BV(NRF24_RF_PWR_LOW) | _BV(NRF24_RF_PWR_HIGH)) ;
    
    // switch uses RAM (evil!)
    if ( level == RF24_PA_MAX )
    {
        setup |= (_BV(NRF24_RF_PWR_LOW) | _BV(NRF24_RF_PWR_HIGH)) ;
    }
    else if ( level == RF24_PA_HIGH )
    {
        setup |= _BV(NRF24_RF_PWR_HIGH) ;
    }
    else if ( level == RF24_PA_LOW )
    {
        setup |= _BV(NRF24_RF_PWR_LOW);
    }
    else if ( level == RF24_PA_MIN )
    {
        // nothing
    }
    else if ( level == RF24_PA_ERROR )
    {
        // On error, go to maximum PA
        setup |= (_BV(NRF24_RF_PWR_LOW) | _BV(NRF24_RF_PWR_HIGH)) ;
    }
    
    nrf24_write_register( NRF24_RF_SETUP, setup ) ;
}

/****************************************************************************/

nrf24_pa_dbm_e nrf24_getPALevel(void)
{
    nrf24_pa_dbm_e result = RF24_PA_ERROR ;
    uint8_t power = nrf24_read_register(NRF24_RF_SETUP) & (_BV(NRF24_RF_PWR_LOW) | _BV(NRF24_RF_PWR_HIGH)) ;
    
    // switch uses RAM (evil!)
    if ( power == (_BV(NRF24_RF_PWR_LOW) | _BV(NRF24_RF_PWR_HIGH)) )
    {
        result = RF24_PA_MAX ;
    }
    else if ( power == _BV(NRF24_RF_PWR_HIGH) )
    {
        result = RF24_PA_HIGH ;
    }
    else if ( power == _BV(NRF24_RF_PWR_LOW) )
    {
        result = RF24_PA_LOW ;
    }
    else
    {
        result = RF24_PA_MIN ;
    }
    
    return result ;
}

/****************************************************************************/

bool nrf24_setDataRate(nrf24_datarate_e speed)
{
    bool result = false;
    uint8_t setup = nrf24_read_register(NRF24_RF_SETUP) ;
    
    // HIGH and LOW '00' is 1Mbs - our default
    wide_band = false ;
    setup &= ~(_BV(NRF24_RF_DR_LOW) | _BV(NRF24_RF_DR_HIGH)) ;
    if( speed == RF24_250KBPS )
    {
        // Must set the RF_DR_LOW to 1; RF_DR_HIGH (used to be RF_DR) is already 0
        // Making it '10'.
        wide_band = false ;
        setup |= _BV( NRF24_RF_DR_LOW ) ;
    }
    else
    {
        // Set 2Mbs, RF_DR (RF_DR_HIGH) is set 1
        // Making it '01'
        if ( speed == RF24_2MBPS )
        {
            wide_band = true ;
            setup |= _BV(NRF24_RF_DR_HIGH);
        }
        else
        {
            // 1Mbs
            wide_band = false ;
        }
    }
    nrf24_write_register(NRF24_RF_SETUP,setup);
    
    // Verify our result
    if ( nrf24_read_register(NRF24_RF_SETUP) == setup )
    {
        result = true;
    }
    else
    {
        wide_band = false;
    }
    
    return result;
}

/****************************************************************************/

nrf24_datarate_e nrf24_getDataRate( void )
{
    nrf24_datarate_e result ;
    uint8_t dr = nrf24_read_register(NRF24_RF_SETUP) & (_BV(NRF24_RF_DR_LOW) | _BV(NRF24_RF_DR_HIGH));
    
    // switch uses RAM (evil!)
    // Order matters in our case below
    if ( dr == _BV(NRF24_RF_DR_LOW) )
    {
        // '10' = 250KBPS
        result = RF24_250KBPS ;
    }
    else if ( dr == _BV(NRF24_RF_DR_HIGH) )
    {
        // '01' = 2MBPS
        result = RF24_2MBPS ;
    }
    else
    {
        // '00' = 1MBPS
        result = RF24_1MBPS ;
    }
    return result ;
}

/****************************************************************************/

void nrf24_setCRCLength(nrf24_crclength_e length)
{
    uint8_t config = nrf24_read_register(NRF24_CONFIG) & ~( _BV(NRF24_CRCO) | _BV(NRF24_EN_CRC)) ;
    
    // switch uses RAM (evil!)
    if ( length == RF24_CRC_DISABLED )
    {
        // Do nothing, we turned it off above.
    }
    else if ( length == RF24_CRC_8 )
    {
        config |= _BV(NRF24_EN_CRC);
    }
    else
    {
        config |= _BV(NRF24_EN_CRC);
        config |= _BV( NRF24_CRCO );
    }
    nrf24_write_register( NRF24_CONFIG, config ) ;
}

/****************************************************************************/

nrf24_crclength_e nrf24_getCRCLength(void)
{
    nrf24_crclength_e result = RF24_CRC_DISABLED;
    uint8_t config = nrf24_read_register(NRF24_CONFIG) & ( _BV(NRF24_CRCO) | _BV(NRF24_EN_CRC)) ;
    
    if ( config & _BV(NRF24_EN_CRC ) )
    {
        if ( config & _BV(NRF24_CRCO) )
            result = RF24_CRC_16;
        else
            result = RF24_CRC_8;
    }
    
    return result;
}

/****************************************************************************/

void nrf24_disableCRC( void )
{
    uint8_t disable = nrf24_read_register(NRF24_CONFIG) & ~_BV(NRF24_EN_CRC) ;
    nrf24_write_register( NRF24_CONFIG, disable ) ;
}

/****************************************************************************/
void nrf24_setRetries(uint8_t delay, uint8_t count)
{
    nrf24_write_register(NRF24_SETUP_RETR, (delay & 0xf) << NRF24_ARD_REG | (count & 0xf) << NRF24_ARC);
}

#endif // NRF24
