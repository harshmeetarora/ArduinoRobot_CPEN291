//
//  BTCentralManager.swift
//  CPEN 291 Project 1
//

import Foundation
import CoreBluetooth

// global BLE central manager - will handle the connection to the peripheral
let btDiscoverySharedInstance = BTCentralManager()

// service and characteristic ID's as defined by Adafruit
// these are Bluetooth Low Energy protocol ID's that allow a central manager
// (in this case the app) choose how it wants to interact with the peripheral device
let BLEServiceUUID = CBUUID(string: "6E400001-B5A3-F393-E0A9-E50E24DCCA9E")
let TXCharacteristicUUID = CBUUID(string: "6E400002-B5A3-F393-E0A9-E50E24DCCA9E")
let RXCharacteristicUUID = CBUUID(string: "6E400003-B5A3-F393-E0A9-E50E24DCCA9E")
let BLEServiceChangedStatusNotification = "kBLEServiceChangedStatusNotification"

class BTCentralManager: NSObject, CBCentralManagerDelegate {
    //MARK: instance variables
    fileprivate var centralManager: CBCentralManager?
    fileprivate var peripheralDevice: CBPeripheral?
    
    var peripheralService: BTPeripheralManager? {
        didSet {
            if let service = self.peripheralService {
                service.discoverServices() // discover the services for this peripheral
            }
        }
    }

    // MARK: Initializer
    override init() {
        super.init()
        //let centralQueue = DispatchQueue(label: "com.raywenderlich", attributes: [])
        centralManager = CBCentralManager(delegate: self, queue: nil)
    }
    
    // scan for nearby BLE peripherals
    func scan() {
        if let central = centralManager {
            central.scanForPeripherals(withServices: [BLEServiceUUID], options: nil)
        }
    }
    
    //MARK: Central Manager Delegate
    
    func centralManager(_ central: CBCentralManager, didDiscover peripheral: CBPeripheral, advertisementData: [String : Any], rssi RSSI: NSNumber) {
        
        
        // if we're already connected to this, return
        if ((self.peripheralDevice == peripheral) && (self.peripheralDevice?.state != CBPeripheralState.disconnected)) {
            print("we're already connected to this")
            return
        }
        
        // If not already connected to a peripheral, then connect to this one
        else {
            // Retain the peripheral before trying to connect
            self.peripheralDevice = peripheral

            // Reset service
            self.peripheralService = nil
            
            // Connect to peripheral
            central.connect(peripheral, options: nil)
        }
    }
    
    func centralManager(_ central: CBCentralManager, didConnect peripheral: CBPeripheral) {
        
        // Create new service class
        if (peripheral == self.peripheralDevice) {
            self.peripheralService = BTPeripheralManager(initWithPeripheral: peripheral)
        }
        
        // Stop scanning for new devices
        central.stopScan()
    }
    
    func centralManager(_ central: CBCentralManager, didDisconnectPeripheral peripheral: CBPeripheral, error: Error?) {
        
        // See if it was our peripheral that disconnected
        if (peripheral == self.peripheralDevice) {
            self.peripheralService = nil;
            self.peripheralDevice = nil;
        }
        
        // Start scanning for new devices
        self.scan()
    }
    
    // reset connections
    func clearConnection() {
        self.peripheralService = nil
        self.peripheralDevice = nil
    }
    
    // Callback function that is called when the app comes into view
    func centralManagerDidUpdateState(_ central: CBCentralManager){
        
        switch (central.state) {
        
        case .poweredOff:
            self.clearConnection()
            
        case .poweredOn:
            self.scan()
            
        case .resetting:
            self.clearConnection()
            
        case .unsupported:
            // BLE not supported
            print("unsupported")
            break
            
        case .unauthorized:
            // BLE not supported
            print("unauthorized")
            break
            
        case .unknown:
            // We will wait for a new event
            print("unknown")
            break
        }
    }
}
