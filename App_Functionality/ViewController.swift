//
//  ViewController.swift
//  CPEN 291 Project 1
//

import UIKit
import QuartzCore

class ViewController: UIViewController {
    
    //MARK: mutable instance variables
    var timerTXDelay: Timer?
    var allowTX = true
    var xAxis: CGFloat = 0
    var yAxis: CGFloat = 0
    var path = UIBezierPath(ovalIn: CGRect(x: 0, y:0, width: 10, height:10))
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
        
        // initialize the axis and the joystick in the app view
        xAxis = outerView.frame.width / 2
        yAxis = outerView.frame.height / 2
        midpoint = joystickSize / 2
        shapeLayer.frame = CGRect(x: (Int(xAxis) - midpoint), y:(Int(yAxis) - midpoint), width: joystickSize, height:joystickSize)
        
        path = UIBezierPath(ovalIn: CGRect(x: 0, y: 0, width: joystickSize, height:joystickSize))
        shapeLayer.path = path.cgPath
        shapeLayer.strokeColor =  UIColor.red.cgColor
        shapeLayer.fillColor = UIColor.red.cgColor
        outerView.layer.addSublayer(shapeLayer)
        
        // flag
        lastTouch = false
        
        // add notifier for this app for the Bluetooth connection
        NotificationCenter.default.addObserver(self, selector: #selector(ViewController.connectionChanged(_:)), name: NSNotification.Name(rawValue: BLEServiceChangedStatusNotification), object: nil)
       
        _ = btDiscoverySharedInstance // reference the global BLE central manager
    }

    override func didReceiveMemoryWarning() {
        super.didReceiveMemoryWarning()
        // Dispose of any resources that can be recreated.
    }
    
    // remove observers
    deinit {
        NotificationCenter.default.removeObserver(self, name: NSNotification.Name(rawValue: BLEServiceChangedStatusNotification), object: nil)
    }
    
    func connectionChanged(_ notification: Notification) {
        // Connection status changed. Indicate on GUI.
        let userInfo = (notification as NSNotification).userInfo as! [String: Bool]
        
        // we are using the main queue by default
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
    
    // Track initial touch location, re-draw the joystick and send to Arduino
    override func touchesBegan(_ touches: Set<UITouch>, with event: UIEvent?) {
        if let touch = touches.first {
            lastTouch = false
            let position = touch.location(in: outerView)
            writePosition(position.x,position2: position.y)
        }
    }
    
    // Continue to track this touch. Re-draw the joystick and send to Arduino
    override func touchesMoved(_ touches: Set<UITouch>, with event: UIEvent?) {
        if let touch = touches.first {
            lastTouch = false
            let position = touch.location(in: outerView)
            writePosition(position.x,position2: position.y)
        }
    }
    
    // Send the final touch to Arduino. X and Y will go back to (0,0) in the center of the screen.
    // The joystick will return to the origin and we will stop the robot.
    override func touchesEnded(_ touches: Set<UITouch>, with event: UIEvent?) {
        lastTouch = true
        self.timerTXDelayElapsed()
        writePosition(CGFloat(0.0),position2: CGFloat(0.0))
    }
    
    // When the app view closes
    override func viewWillDisappear(_ animated: Bool) {
        super.viewWillDisappear(animated)
        
        self.stopTimerTXDelay()
    }
    
    // Where the magic happens. We take the x and y coordinate from the view and 
    // pass it via Bluetooth to the Arduino
    private func writePosition(_ position1: CGFloat, position2: CGFloat) {
        
        // if our timer is not up, return
        if !allowTX {
            return
        }
        
        // if there is an error finding the peripheral, return
        guard let bleService = btDiscoverySharedInstance.peripheralService else {
            print("nothing from peripheral")
            return
        }
        
        // Initialize x and y to normalize the touch positions.
        // We will send x and y values relative to the centre of the screen.
        // the default position is relative to the top left corner of the phone screen.
        var x = CGFloat(0.0)
        var y = CGFloat(0.0)
        var negativeX = UInt8(0) // negative flag
        var negativeY = UInt8(0) // negative flag
        
        
        // only convert if we're not writing a touchEnded value
        if (position1 != 0.0 && position2 != 0.0) {
            x = position1 - xAxis // relative to x axis
            y = yAxis - position2 // relative to y axis
        }
        
        if (x < 0) {
            negativeX = 1 // set flag
            x = -x // we can send unsigned integers only
        }
        
        if (y < 0) {
            negativeY = 1
            y = -y
        }
        
        // We'll set the max value to of x and y to 250 because we will use the value 255
        // as our negative flag (data is sent in 1 byte chunks, so 255 is the actual max value).
        if (x >= 250) {
            x = 250
        }
        
        if (y >= 250) {
            y = 250
        }
        
        // update joystick circle and write to robot
        redrawCircle(position1, y: position2)
        bleService.writeToRobot(negativeX, negativeY: negativeY, positionx: UInt8(x), positiony: UInt8(y))
        
        // Start delay timer
        allowTX = false
        if timerTXDelay == nil {
            timerTXDelay = Timer.scheduledTimer(timeInterval: 0.1, target: self, selector: #selector(ViewController.timerTXDelayElapsed), userInfo: nil, repeats: false)
        }
    }
    
    // Draws a red circle at the given coordinate location on-screen
    private func redrawCircle(_ x: CGFloat, y: CGFloat) {
        // remove current circle
        path.removeAllPoints()
        outerView.setNeedsDisplay()
        shapeLayer.removeAllAnimations()
        shapeLayer.removeFromSuperlayer()
        
        // when the user removes their finger, reposition the circle in the middle of the screen
        if (lastTouch) {
            shapeLayer.frame = CGRect(x: (Int(xAxis) - midpoint), y:(Int(yAxis) - midpoint), width: joystickSize, height:joystickSize)
        } else {
            shapeLayer.frame = CGRect(x: (Int(x) - midpoint), y:(Int(y) - midpoint), width: joystickSize, height:joystickSize)
        }
        
        // redraw
        path = UIBezierPath(ovalIn: CGRect(x: 0, y: 0, width: joystickSize, height:joystickSize))
        shapeLayer.path = path.cgPath
        shapeLayer.strokeColor =  UIColor.red.cgColor
        shapeLayer.fillColor = UIColor.red.cgColor
        outerView.layer.addSublayer(shapeLayer)
    }
    
    /* === timer functions === */
    func timerTXDelayElapsed() {
        self.allowTX = true
        self.stopTimerTXDelay()
    }
    
    func stopTimerTXDelay() {
        if self.timerTXDelay == nil {
            return
        }
        
        timerTXDelay?.invalidate()
        self.timerTXDelay = nil
    }
    

}

