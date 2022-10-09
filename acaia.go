package main

import (
	"context"
	"fmt"
	"log"
	"sync"
	"time"

	"github.com/go-ble/ble"
	"github.com/go-ble/ble/examples/lib/dev"
)

var (
	heartbeatPacket     = []byte{0xef, 0xdd, 0x00, 0x02, 0x00, 0x02, 0x00}
	notificationRequest = []byte{0xef, 0xdd, 0x0c, 0x09, 0x00, 0x01, 0x01, 0x02, 0x02, 0x05, 0x03, 0x04, 0x15, 0x06}
	scaleId             = []byte{0xef, 0xdd, 0x0b, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x9a, 0x6d}
	tarePacket          = []byte{0xef, 0xdd, 0x04, 0x00, 0x00, 0x00}
	scaleService        = ble.UUID16(0x1820)
	scaleCharacteristic = ble.NewCharacteristic(ble.UUID16(0x2a80))
)

func main() {
	fmt.Println("Starting")
	d, err := dev.NewDevice("default")
	if err != nil {
		log.Fatalf("can't new device : %s", err)
	}
	ble.SetDefaultDevice(d)

	filter := func(a ble.Advertisement) bool {
		svcs := a.Services()
		if len(svcs) > 0 {
			if svcs[0].Equal(scaleService) {
				return true
			}
		}
		return false
	}

	// Scan for specified durantion, or until interrupted by user.
	ctx := ble.WithSigHandler(context.WithTimeout(context.Background(), 3*time.Second))
	cln, err := ble.Connect(ctx, filter)
	if err != nil {
		log.Fatalf("can't connect : %s", err)
	}

	// Make sure we had the chance to print out the message.
	done := make(chan struct{})
	// Normally, the connection is disconnected by us after our exploration.
	// However, it can be asynchronously disconnected by the remote peripheral.
	// So we wait(detect) the disconnection in the go routine.
	go func() {
		<-cln.Disconnected()
		fmt.Printf("[ %s ] is disconnected \n", cln.Addr())
		close(done)
	}()

	fmt.Printf("Discovering profile...\n")
	p, err := cln.DiscoverProfile(true)
	if err != nil {
		log.Fatalf("can't discover profile: %s", err)
	}

	c := p.FindCharacteristic(scaleCharacteristic)

	// Subscribe

	err = cln.Subscribe(c, false, nty)
	if err != nil {
		panic(err)
	}

	writeIdPacket(cln, c)
	//startHeartbeat(cln, c)
	startWeight(cln, c)

	//time.Sleep(1 * time.Second)
	//writeTarePacket(cln, c)

	var wg sync.WaitGroup
	wg.Add(1)
	wg.Wait()

}

func writeTarePacket(cln ble.Client, c *ble.Characteristic) {
	fmt.Println("Writing tare packet...")
	cln.WriteCharacteristic(c, tarePacket, true)
}

func startWeight(cln ble.Client, c *ble.Characteristic) {
	fmt.Println("Writing notification request packet...")
	cln.WriteCharacteristic(c, notificationRequest, true)
}

func writeIdPacket(cln ble.Client, c *ble.Characteristic) {
	fmt.Println("Writing id packet...")
	cln.WriteCharacteristic(c, scaleId, true)
}

func doHeartbeat(cln ble.Client, c *ble.Characteristic) {
	for {
		fmt.Println("Writing heartbeat...")
		cln.WriteCharacteristic(c, heartbeatPacket, true)
		time.Sleep(2 * time.Second)
	}
}

func nty(req []byte) {
	if len(req) < 3 {
		return
	}
	switch {
	case req[0] == 8 && req[1] == 5:
		// Weight reading
		weight := uint32(req[2]) + uint32(req[3])<<8 + uint32(req[4])<<16 + uint32(req[5])<<24
		fmt.Println("WEIGHT     ", float32(weight)/100)
	case req[0] == 239 && req[1] == 221:
		// General heartbeat
	case req[0] == 14 && req[1] == 8 && req[2] == 9:
		// Timer reset
		fmt.Println("TIMER RESET")
	case req[0] == 14 && req[1] == 8 && req[2] == 10:
		duration := fmt.Sprintf("%dm%d.%ds", req[4], req[5], req[6])
		t, _ := time.ParseDuration(duration)
		fmt.Println("TIMER STOP", t)
	case req[0] == 10 && req[1] == 8 && req[2] == 8:
		fmt.Println("TIMER START")
	case req[0] == 10 && req[1] == 8 && req[2] == 0:
		fmt.Println("TARE")
	case req[0] == 10 && req[1] == 5:
		// General status update
	case req[0] == 12 && req[1] == 5:
		// Timer update
		duration := fmt.Sprintf("%dm%d.%ds", req[9], req[10], req[11])
		t, _ := time.ParseDuration(duration)
		fmt.Println("TIMER     ", t)
	default:
		fmt.Println(req[0], req[1], req)
	}
}

func startHeartbeat(cln ble.Client, c *ble.Characteristic) {
	go doHeartbeat(cln, c)
}

/*
func readWeight(cln ble.Client, c *ble.Characteristic) error {
	hnd := func(req []byte) {
		fmt.Printf("Notified: %q [ % X ]\n", string(req), req)
	}
	err := cln.Subscribe(c, false, hnd)
	if err != nil {
		log.Fatalf("subscribe failed: %s", err)
	}
	cln.WriteCharacteristic(c, startWeightPacket, false)
	return nil
}
*/
