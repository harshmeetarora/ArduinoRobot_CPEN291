//
//  ViewController.swift
//  RobotJoystick
//
//  Created by James Asefa on 2017-02-17.
//  Copyright © 2017 James Asefa. All rights reserved.
//

import UIKit
import QuartzCore

class ViewController: UIViewController {
    
    //MARK: mutable instance variables
    var timerTXDelay: Timer?
    var allowTX = true
    var lastPositionx: CGFloat = 255.0
    var lastPositiony: CGFloat = 255.0
    var xAxis: CGFloat = 0
    var yAxis: CGFloat = 0
    var path = UIBezierPath(ovalIn: CGRect(x: 0, y:0, width: 10, height:10))
    var validTouch = Bool()
    var lastTouch = Bool()
    var midpoint = 0
    
    //MARK: immutable instance variables
    let joystickSize = 100
    let shapeLayer = CAShapeLayer() // for drawings
    
    //MARK: Properties
    @IBOutlet var outerView: UIView!
    
    //MARK: View functions
    override func viewDidLoad() {
        super.viewDidLoad()
        
        xAxis = outerView.frame.width / 2
        yAxis = outerView.frame.height / 2
        midpoint = joystickSize / 2
        shapeLayer.frame = CGRect(x: (Int(xAxis) - midpoint), y:(Int(yAxis) - midpoint), width: joystickSize, height:joystickSize)
        
        path = UIBezierPath(ovalIn: CGRect(x: 0, y: 0, width: joystickSize, height:joystickSize))
        shapeLayer.path = path.cgPath
        shapeLayer.strokeColor =  UIColor.red.cgColor
        shapeLayer.fillColor = UIColor.red.cgColor
        outerView.layer.addSublayer(shapeLayer)
        
        validTouch = false
        lastTouch = false
        
        NotificationCenter.default.addObserver(self, selector: #selector(ViewController.connectionChanged(_:)), name: NSNotification.Name(rawValue: BLEServiceChangedStatusNotification), object: nil)
       _ = btDiscoverySharedInstance
    }

    override func didReceiveMemoryWarning() {
        super.didReceiveMemoryWarning()
        // Dispose of any resources that can be recreated.
    }
    
    deinit {
        NotificationCenter.default.removeObserver(self, name: NSNotification.Name(rawValue: BLEServiceChangedStatusNotification), object: nil)
    }
    
    func connectionChanged(_ notification: Notification) {
        // Connection status changed. Indicate on GUI.
        let userInfo = (notification as NSNotification).userInfo as! [String: Bool]
        
        DispatchQueue.main.async(execute: {
            // print connection status
            if let isConnected: Bool = userInfo["isConnected"] {
                if isConnected {
                    print("Connected!")
                } else {
                    print("Not Connected!")
                }
            }
        });
    }
    
    override func touchesBegan(_ touches: Set<UITouch>, with event: UIEvent?) {
        if let touch = touches.first {
            lastTouch = false
            let position = touch.location(in: outerView)
            writePosition(position.x,position2: position.y)
        }
    }
    
    override func touchesMoved(_ touches: Set<UITouch>, with event: UIEvent?) {
        if let touch = touches.first {
            lastTouch = false
            let position = touch.location(in: outerView)
            writePosition(position.x,position2: position.y)
        }
    }
    
    // want to send speed of 0 and reset joystick to middle
    override func touchesEnded(_ touches: Set<UITouch>, with event: UIEvent?) {
        lastTouch = true
        self.timerTXDelayElapsed()
        writePosition(CGFloat(0.0),position2: CGFloat(0.0))
    }
    
    override func viewWillDisappear(_ animated: Bool) {
        super.viewWillDisappear(animated)
        
        self.stopTimerTXDelay()
    }
    
    private func writePosition(_ position1: CGFloat, position2: CGFloat) {
        
        if !allowTX {
            return
        }
        
        guard let bleService = btDiscoverySharedInstance.peripheralService else {
            print("nothing from peripheral")
            return
        }
        
        if (shapeLayer.contains(CGPoint(x: position1, y: position1))){
            validTouch = true
        } else {
            validTouch = false
        }
        
        // initialize x and y
        var x = CGFloat(0.0)
        var y = CGFloat(0.0)
        
        
        // only convert if we're not writing a touchEnded value
        if (position1 != 0.0 && position2 != 0.0) {
            x = position1 - xAxis // relative to x axis
            y = yAxis - position2 // relative to y axis
        }
        
        if (x < 0) {
            bleService.writeLeadingNegativeByteToRobot()
            x = -x
        }
        
        if (y < 0) {
            bleService.writeLeadingNegativeByteToRobot()
            y = -y
        }
        
        if (x >= 250) {
            x = 250
        }
        
        if (y >= 250) {
            y = 250
        }
        
        
        //print(validTouch)
        //print(position1)
        //print(position2)
        
        redrawCircle(position1, y: position2)
        bleService.writeToRobot(UInt8(x), positiony: UInt8(y))
        lastPositionx = x;
        lastPositiony = y;
        
        // Start delay timer
        allowTX = false
        if timerTXDelay == nil {
            timerTXDelay = Timer.scheduledTimer(timeInterval: 0.1, target: self, selector: #selector(ViewController.timerTXDelayElapsed), userInfo: nil, repeats: false)
        }
    }
    
    func redrawCircle(_ x: CGFloat, y: CGFloat) {
        path.removeAllPoints()
        outerView.setNeedsDisplay()
        shapeLayer.removeAllAnimations()
        shapeLayer.removeFromSuperlayer()
        if (lastTouch) {
            shapeLayer.frame = CGRect(x: (Int(xAxis) - midpoint), y:(Int(yAxis) - midpoint), width: joystickSize, height:joystickSize)
        } else {
            shapeLayer.frame = CGRect(x: (Int(x) - midpoint), y:(Int(y) - midpoint), width: joystickSize, height:joystickSize)
        }
        
        path = UIBezierPath(ovalIn: CGRect(x: 0, y: 0, width: joystickSize, height:joystickSize))
        shapeLayer.path = path.cgPath
        shapeLayer.strokeColor =  UIColor.red.cgColor
        shapeLayer.fillColor = UIColor.red.cgColor
        outerView.layer.addSublayer(shapeLayer)
    }
    
    func timerTXDelayElapsed() {
        self.allowTX = true
        self.stopTimerTXDelay()
        
        // Send current slider position
        //self.writePosition(0.0, position2: 0.0)
    }
    
    func stopTimerTXDelay() {
        if self.timerTXDelay == nil {
            return
        }
        
        timerTXDelay?.invalidate()
        self.timerTXDelay = nil
    }
    

}

