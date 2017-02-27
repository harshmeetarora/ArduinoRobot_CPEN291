//
//  BTCentralManager.swift
//  RobotJoystick
//
//  Created by James Asefa on 2017-02-23.
//  Copyright © 2017 James Asefa. All rights reserved.
//

import Foundation
import CoreBluetooth

let btDiscoverySharedInstance = BTCentralManager()

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
                service.discoverServices()
            }
        }
    }

    // MARK: Initializer
    override init() {
        super.init()
        //let centralQueue = DispatchQueue(label: "com.raywenderlich", attributes: [])
        centralManager = CBCentralManager(delegate: self, queue: nil)
    }
    
    func scan() {
        if let central = centralManager {
            central.scanForPeripherals(withServices: [BLEServiceUUID], options: nil)
        }
    }
    
    //MARK: Central Manager Delegate
    
    func centralManager(_ central: CBCentralManager, didDiscover peripheral: CBPeripheral, advertisementData: [String : Any], rssi RSSI: NSNumber) {
        
        //let svcIDs = advertisementData[CBAdvertisementDataServiceUUIDsKey] as? NSArray
        //print(peripheral)
        //print(svcIDs)
        
        
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
    
    func clearConnection() {
        self.peripheralService = nil
        self.peripheralDevice = nil
    }
    
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
