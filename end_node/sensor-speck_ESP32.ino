/*******************************************************************************
 * Copyright (c) 2020 Pablo Rodríguez Martín
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 *
 * This example sends a valid LoRaWAN packet with payload containing
 * the humidity and the temperature, using frequency and encryption settings
 * matching those of ChirpStack, ABP (Activation-by-personalisation)where a 
 * DevAddr and Session keys are preconfigured (unlike OTAA, where a DevEUI and
 * application key is configured, while the DevAddr and session keys are
 * assigned/generated in the over-the-air-activation procedure)..
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!
 *
 * To use this sketch, first register your application and device with
 * the things network, to set or generate a DevAddr, NwkSKey and
 * AppSKey. Each device should have their own unique values for these
 * fields.
 *
 *******************************************************************************/

#include <SSD1306.h> 
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <DHT.h>

SSD1306 display(0x3C, 4, 15);
const int pinHumidity = 34;
const int pinTemperature = 23;

DHT dht(pinTemperature, DHT11);
int prime = 0;
int prime2 = 0;
int humidity = 0;
float temperature = 0;
int counter = 0;
boolean sent = 0;
float prime3 = 0;

#define CHANNEL 0  // Channel used within this demo.
#define SF 7 // SF sed within this demo

// LoRaWAN NwkSKey, network session key
static const PROGMEM u1_t NWKSKEY[16] = { 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C };

// LoRaWAN AppSKey, application session key
static const u1_t PROGMEM APPSKEY[16] = { 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C };

// LoRaWAN end-device address (DevAddr)
static const u4_t DEVADDR = 0x03FF0001 ; // <-- Change this address for every node!

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer than 60 due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 179; // 350 tx - 1 rx (3 min)

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,
    .dio = {26, 33, 32},
};

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            break;
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            sent = 1;
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
         default:
            Serial.println(F("Unknown event"));
            break;
    }
}

void do_send(osjob_t* j){
    sent = 0;
    prime = analogRead(pinHumidity);
    prime2 = map(prime, 0, 1319, 0, 100); // 16 -> 0.1V  ,  3360 -> 2.8V
    humidity = constrain(prime2, 0, 100);
    Serial.println(prime);
    //prime = random(0, 52);
    temperature = (dht.readTemperature(true)-32)*5/9;
    Serial.println((int)temperature);
    
    byte payload[4];  // Data to Byte format (2 Bytes/value)
    payload[0] = highByte(humidity);
    payload[1] = lowByte(humidity);
    payload[2] = highByte((int)temperature);
    payload[3] = lowByte((int)temperature);
    counter++;
    
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, payload, sizeof(payload), 0);
        Serial.println("Frequency band: " + String(LMIC.freq) + " Hz");
        Serial.println("Packet number: " + String(counter) + " ,   Humidity: " + String(humidity) + "  ,   Temperature: " + String(temperature));
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
    Serial.begin(115200);
    Serial.println(F("Starting"));

    pinMode(16,OUTPUT);
    digitalWrite(16, LOW);    // set GPIO16 low to reset OLED
    delay(50); 
    digitalWrite(16, HIGH); // while OLED is running, must set GPIO16 in high
    display.init();
    display.flipScreenVertically();
    display.display();
    //pinMode(34,INPUT);
    dht.begin();

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.
    #ifdef PROGMEM
    // On AVR, these values are stored in flash and only copied to RAM
    // once. Copy them to a temporary buffer here, LMIC_setSession will
    // copy them into a buffer of its own again.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
    #else
    // If not running an AVR with PROGMEM, just use the arrays directly
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
    #endif

    #if defined(CFG_eu868)
    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.
    // NA-US channels 0-71 are configured automatically
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
    for (uint8_t i = 0; i < 9; i++){ // Discards rest of channels to avoid dropping
      if (i != CHANNEL){
        LMIC_disableChannel(i);
      }
    }
    // TTN defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.
    #elif defined(CFG_us915)
    // NA-US channels 0-71 are configured automatically
    // but only one group of 8 should (a subband) should be active
    // TTN recommends the second sub band, 1 in a zero based count.
    // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
    LMIC_selectSubBand(1);
    #endif

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // Use of 869525000 Hz channel for its RX2 window.
    LMIC.dn2Dr = EU868_F6;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    if(SF==7){ 
      LMIC_setDrTxpow(DR_SF7,14); // 14dB
      Serial.println("Spreading Factor: 7");
    }
    if(SF==8){ 
      LMIC_setDrTxpow(DR_SF8,14); // 14dB
      Serial.println("Spreading Factor: 8");
    }
    if(SF==9){ 
      LMIC_setDrTxpow(DR_SF9,14); // 14dB
      Serial.println("Spreading Factor: 9");
    }
    if(SF==10){ 
      LMIC_setDrTxpow(DR_SF10,14); // 14dB
      Serial.println("Spreading Factor: 10");
    }
    if(SF==11){ 
      LMIC_setDrTxpow(DR_SF11,14); // 14dB
      Serial.println("Spreading Factor: 11");
    }
    if(SF==12){ 
      LMIC_setDrTxpow(DR_SF12,14); // 14dB
      Serial.println("Spreading Factor: 12");
    }
    if(SF < 7 || SF > 12){
      LMIC_setDrTxpow(DR_SF7,14); // 14dB -- SF7 DEFAULT
      Serial.println("ERROR: SF NOT AVAILABLE");
    }

    // Start job
    do_send(&sendjob);
}

void loop() {
    os_runloop_once();
  
    display.clear();
    display.setFont(ArialMT_Plain_10);
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.drawString(0, 0, "Sending packet: ");
    display.drawString(80, 0, String(counter));
    if(sent)
      display.drawString(110, 0, "OK");
    display.setFont(ArialMT_Plain_16);
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.drawString(0, 20, "Humidity: ");
    display.drawString(70, 20, String(humidity));
    display.drawString(100, 20, "\%");
    display.drawString(0, 42, "Temp: ");
    display.drawString(70, 42, String((int)temperature));
    display.drawString(95, 42, "\ºC");
    display.display();
    
}
