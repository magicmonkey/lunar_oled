#include <bluefruit.h>

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &Wire);

BLEClientService        ble_svc = BLEClientService(0x1820);
BLEClientCharacteristic ble_chr = BLEClientCharacteristic(0x2a80);

uint8_t idPacket[] {0xef, 0xdd, 0x0b, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x9a, 0x6d};
uint8_t startWeight[] {0xef, 0xdd, 0x0c, 0x09, 0x00, 0x01, 0x01, 0x02, 0x02, 0x05, 0x03, 0x04, 0x15, 0x06};

uint8_t heartbeatPacket[] {0xef, 0xdd, 0x00, 0x02, 0x00, 0x02, 0x00};

uint32_t currentWeight;
uint8_t currentMins;
uint8_t currentSecs;
bool timerRunning = false;
bool connected = false;

void setup() {
	Serial.begin(115200);
	//while ( !Serial ) delay(10);

	display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
	display.setTextColor(WHITE);
	display.setTextSize(2);

	display.fillScreen(0);
	display.display();

	Bluefruit.begin(0, 1);

	Bluefruit.setName("AcaiaFlow");
	Bluefruit.autoConnLed(false);
	//Bluefruit.setConnLedInterval(1000);

	ble_svc.begin();
	ble_chr.setNotifyCallback(scale_notify_callback);
	ble_chr.begin();

	// Callbacks for Central
	Bluefruit.Central.setDisconnectCallback(disconnect_callback);
	Bluefruit.Central.setConnectCallback(connect_callback);

	Bluefruit.Scanner.setRxCallback(scan_callback);
	Bluefruit.Scanner.restartOnDisconnect(true);
	Bluefruit.Scanner.setInterval(160, 80); // in unit of 0.625 ms
	Bluefruit.Scanner.filterUuid(ble_svc.uuid);
	Bluefruit.Scanner.useActiveScan(true);
	Bluefruit.Scanner.start(0);

}

void scan_callback(ble_gap_evt_adv_report_t* report) {
	Bluefruit.Central.connect(report);
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
	(void) conn_handle;
	(void) reason;
	connected = false;
	Serial.printf("XXX Disconnected (%d), reason = 0x", conn_handle);
	Serial.println(reason, HEX);
	delay(2000);
}

void connect_callback(uint16_t conn_handle) {
	if ( !ble_svc.discover(conn_handle) ) {
		Serial.println("XXX Service discovery failed");
		Bluefruit.disconnect(conn_handle);
		return;
	}

	if (!ble_chr.discover()) {
		Serial.println("XXX Characteristic discovery failed");
		Bluefruit.disconnect(conn_handle);
		return;
	}
	connected = true;

	ble_chr.enableNotify();
	ble_chr.write(idPacket, 20);
	ble_chr.write(startWeight, 14);

}

void heartbeat() {
	ble_chr.write(heartbeatPacket, 7);
}

uint32_t weightSamples[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

void loop() {
	unsigned long time = millis();
	static unsigned long timeSinceLastHeartbeat = millis();
	
	display.fillScreen(0);

	if (connected) {
		if (time - timeSinceLastHeartbeat > 1000) {
			heartbeat();
			timeSinceLastHeartbeat = time;
		}

		weightSamples[10] = weightSamples[9];
		weightSamples[9] = weightSamples[8];
		weightSamples[8] = weightSamples[7];
		weightSamples[7] = weightSamples[6];
		weightSamples[6] = weightSamples[5];
		weightSamples[5] = weightSamples[4];
		weightSamples[4] = weightSamples[3];
		weightSamples[3] = weightSamples[2];
		weightSamples[2] = weightSamples[1];
		weightSamples[1] = weightSamples[0];
		weightSamples[0] = currentWeight;

		//Serial.printf("%2.1f", (float)currentWeight/100.0);


		// Current weight
		display.setTextSize(1);
		display.setCursor(0, 24);
		display.printf("%2.1f", (float)currentWeight/100.0);

		// Rate of weight change
		display.setTextSize(2);
		display.setCursor(0, 0);
		display.printf("%2.1f", (float)((int32_t)weightSamples[0] - (int32_t)weightSamples[5])/100.0);

		// Timer
		display.setTextSize(1);
		display.setCursor(64, 24);
		display.printf("%d:%02d", currentMins, currentSecs);

		display.display();

		delay(100);
	} else {
		display.display();
		delay(1000);
	}
}

void scale_notify_callback(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len) {
	//Serial.println("--- Scale notify");
	//Serial.printf("%d\n", len);

/*
	        case req[0] == 8 && req[1] == 5:
                // Weight reading
                weight := uint32(req[2]) + uint32(req[3])<<8 + uint32(req[4])<<16 + uint32(req[5])<<24
                fmt.Println("WEIGHT     ", float32(weight)/100)
                */

	if (len < 3) {
		return;
	}

	if (data[0] == 8 && data[1] == 5) {
		currentWeight = ((uint32_t)data[2]) | ((uint32_t)data[3])<<8 | ((uint32_t)data[4])<<16 | ((uint32_t)data[5])<<24;
		//Serial.printf("WEIGHT %2.1f\n", (float)weight/100.0);
	}

	// Timer start
	if (data[0] == 10 && data[1] == 8 && data[1] == 8) {
		timerRunning = true;
	}
	// Timer reset
	if (data[0] == 14 && data[1] == 8 && data[1] == 9) {
		currentMins = 0;
		currentSecs = 0;
	}
	// Timer stop
	if (data[0] == 14 && data[1] == 8 && data[1] == 10) {
		timerRunning = false;
	}
	if (data[0] == 12 && data[1] == 5) {
		currentMins = data[9];
		currentSecs = data[10];
	}
/*
	if (len == 10) {
		for (int i=0; i<len; i++) {
			Serial.printf(" - %d\n", (int)data[i]);
		}
	}
	*/
}

