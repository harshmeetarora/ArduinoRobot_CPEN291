//
//  BTPeripheralManager.swift
//  RobotJoystick
//
//  Created by James Asefa on 2017-02-23.
//  Copyright © 2017 James Asefa. All rights reserved.
//

import Foundation
import CoreBluetooth

class BTPeripheralManager: NSObject, CBPeripheralDelegate {
    //MARK: instance variables
    var peripheral: CBPeripheral?
    var txCharacteristic: CBCharacteristic?
    
    //MARK: initializers
    init(initWithPeripheral peripheral: CBPeripheral) {
        super.init()
        
        self.peripheral = peripheral
        self.peripheral?.delegate = self
    }
    
    deinit {
        self.reset()
    }
    
    
    // MARK: Peripheral Delegate
    
    func peripheral(_ peripheral: CBPeripheral, didDiscoverServices error: Error?) {
        //let characteristicUUIDs: [CBUUID] = [SpeedUUID, DirectionUUID]
        
        if ((peripheral != self.peripheral) || (error != nil)) {
            // Wrong peripheral or we have an error
            return
        }
        
        if ((peripheral.services == nil) || (peripheral.services!.count == 0)) {
            // No services available
            return
        }
        
        for service in peripheral.services! {
            if service.uuid == BLEServiceUUID {
                peripheral.discoverCharacteristics(nil, for: service)
            }
        }
    }
    
    func peripheral(_ peripheral: CBPeripheral, didDiscoverCharacteristicsFor service: CBService, error: Error?) {
        
        if ((peripheral != self.peripheral) || (error != nil)) {
            self.updateConnectionStatus(false)
            return // wrong peripheral or error discovering characteristics
        }
        
        guard let characteristics = service.characteristics else {
            self.updateConnectionStatus(false)
            return // error unwrapping or no characteristics available
        }
        
        for characteristic in characteristics {
            print(String(describing: characteristic.uuid))
            if characteristic.uuid == TXCharacteristicUUID {
                self.txCharacteristic = (characteristic)
                peripheral.setNotifyValue(true, for: characteristic)
                
                // Send notification that Bluetooth is connected and all required characteristics are discovered
                self.updateConnectionStatus(true)
            }
        }
        
        // Send notification that Bluetooth is connected and all required characteristics are discovered
        self.updateConnectionStatus(true)
    }
    
    func writeToRobot(_ positionx: UInt8, positiony: UInt8) {
        if (txCharacteristic == nil){
            print("Unable to write data without txcharacteristic")
            return
        }
        
        var writeType:CBCharacteristicWriteType
        
        if (txCharacteristic!.properties.rawValue & CBCharacteristicProperties.writeWithoutResponse.rawValue) != 0 {
            
            writeType = CBCharacteristicWriteType.withoutResponse
            
        }
            
        else if ((txCharacteristic!.properties.rawValue & CBCharacteristicProperties.write.rawValue) != 0){
            
            writeType = CBCharacteristicWriteType.withResponse
        }
            
        else{
            print("Unable to write data without characteristic write property")
            return
        }
        
        if let txCharacteristic = self.txCharacteristic {
            let data = Data(bytes: [positionx, positiony])
            self.peripheral?.writeValue(data, for: txCharacteristic, type: writeType)
        }
    }
    
    // write the value 255 to robot signifying that the next byte
    // represents a negative number
    func writeLeadingNegativeByteToRobot() {
        if (txCharacteristic == nil){
            print("Unable to write data without txcharacteristic")
            return
        }
        
        var writeType:CBCharacteristicWriteType
        
        if (txCharacteristic!.properties.rawValue & CBCharacteristicProperties.writeWithoutResponse.rawValue) != 0 {
            
            writeType = CBCharacteristicWriteType.withoutResponse
            
        }
            
        else if ((txCharacteristic!.properties.rawValue & CBCharacteristicProperties.write.rawValue) != 0){
            
            writeType = CBCharacteristicWriteType.withResponse
        }
            
        else{
            print("Unable to write data without characteristic write property")
            return
        }
        
        if let txCharacteristic = self.txCharacteristic {
            let data = Data(bytes: [255])
            self.peripheral?.writeValue(data, for: txCharacteristic, type: writeType)
        }
    }
    
    func reset() {
        if peripheral != nil {
            peripheral = nil
        }
        
        // update central with status
        self.updateConnectionStatus(false)
    }
    
    // notify outside world when we connect or disconnect
    func updateConnectionStatus(_ connected: Bool) {
        let connectionPair = ["isConnected": connected]
        NotificationCenter.default.post(name: Notification.Name(rawValue: BLEServiceChangedStatusNotification), object: self, userInfo: connectionPair)
    }
    
    // discover the services from the Bluetooth peripheral
    func discoverServices() {
        self.peripheral?.discoverServices([BLEServiceUUID])
    }

}
