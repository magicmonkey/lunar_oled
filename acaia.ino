#include <bluefruit.h>

BLEClientService        ble_svc = BLEClientService(0x1820);
BLEClientCharacteristic ble_chr = BLEClientCharacteristic(0x2a80);

uint8_t idPacket[] {0xef, 0xdd, 0x0b, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x9a, 0x6d};
uint8_t startWeight[] {0xef, 0xdd, 0x0c, 0x09, 0x00, 0x01, 0x01, 0x02, 0x02, 0x05, 0x03, 0x04, 0x15, 0x06};

uint8_t heartbeatPacket[] {0xef, 0xdd, 0x00, 0x02, 0x00, 0x02, 0x00};

void setup() {
	Serial.begin(115200);
	while ( !Serial ) delay(10);
	Serial.println("--- Begin");

	Bluefruit.begin(0, 1);

	Bluefruit.setName("AcaiaFlow");
	Bluefruit.autoConnLed(true);
	Bluefruit.setConnLedInterval(1000);

	// Callbacks for Central
	Bluefruit.Central.setDisconnectCallback(disconnect_callback);
	Bluefruit.Central.setConnectCallback(connect_callback);

	Bluefruit.Scanner.setRxCallback(scan_callback);
	Bluefruit.Scanner.restartOnDisconnect(true);
	Bluefruit.Scanner.setInterval(160, 80); // in unit of 0.625 ms
	Bluefruit.Scanner.filterUuid(ble_svc.uuid);
	Bluefruit.Scanner.useActiveScan(true);
	Bluefruit.Scanner.start(0);

	ble_svc.begin();

	ble_chr.setNotifyCallback(scale_notify_callback);
	ble_chr.begin(&ble_svc);

	Serial.println("--- Setup done");
}

void scan_callback(ble_gap_evt_adv_report_t* report) {
	Serial.println("--- scan_callback");
	// Should only be called for the relevant device as it is
	// already filtered by the scanner
	Bluefruit.Central.connect(report);
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
	(void) conn_handle;
	(void) reason;
	Serial.printf("XXX Disconnected (%d), reason = 0x", conn_handle);
	Serial.println(reason, HEX);
}

void connect_callback(uint16_t conn_handle) {
	Serial.printf("--- connect_callback (%d)\n", conn_handle);

	Serial.println("--- Requesting pairing...");

	Serial.println("--- Doing service discovery...");
	if ( !ble_svc.discover(conn_handle) ) {
		Serial.println("XXX Service discovery failed");
		Bluefruit.disconnect(conn_handle);
		return;
	}

	Serial.println("--- Doing characteristic discovery...");
	if (!ble_chr.discover()) {
		Serial.println("XXX Characteristic discovery failed");
		Bluefruit.disconnect(conn_handle);
		return;
	}

	Serial.println("Enabling notify...");
	if ( ble_chr.enableNotify() ) {
		Serial.println("--- Ready to receive scale measurement values");
	} else {
		Serial.println("XXX Couldn't enable notify for scale measurements values");
	}
	delay(700);

	Serial.println("Writing ID packet...");
	ble_chr.write(idPacket, 20);
	delay(700);

	Serial.println("Writing startWeight packet...");
	ble_chr.write(startWeight, 14);
	delay(700);

}

void heartbeat() {
	Serial.println("--- Heartbeat");
	ble_chr.write(heartbeatPacket, 7);
}

void loop() {
	Serial.println("--- Loop");
	//heartbeat();
	delay(1000);
}

void scale_notify_callback(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len) {
	Serial.println("--- Scale notify");
}
