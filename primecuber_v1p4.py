#-----------------------------------------------------------------------------
# Title:        PrimeCuber
#
# Author:    David Gilday
#
# Copyright:    (C) 2020 David Gilday
#
# Website:    http://PrimeCuber.com
#
# Version:    v1p4
#
# Modified:    $Date: 2020-12-04 18:06:26 +0000 (Fri, 04 Dec 2020) $
#
# Revision:    $Revision: 7785 $
#
# Usage:
#
#This software may be used for any non-commercial purpose providing
#that the original author is acknowledged.
#
# Disclaimer:
#
#This software is provided 'as is' without warranty of any kind, either
#express or implied, including, but not limited to, the implied warranties
#of fitness for a purpose, or the warranty of non-infringement.
#
#-----------------------------------------------------------------------------
# Purpose:    Main program for PrimeCuber robot Rubik's Cube solver
#-----------------------------------------------------------------------------

import gc, time, hub
gc.collect()

import pcsolver_v1p4

def trace(msg):
    if False:
        gc.collect()
        print("TRACE: "+msg+" mem="+str(gc.mem_free()))

trace("module primecuber_v1p4")

trace("class primecuber")
class primecuber():

    # MD: original -> turn_ratio = int(36/12)
    # MD: 36 gear used for scanner arm. Replaced with 24/8 (ratio also 3)
    turn_ratio  = int(24/8)

    def __init__(self):
        self.count    = 0
        # MD: original = 38, no change
        self.scan_speed = 38
        self.slower    = False
        self.c        = pcsolver_v1p4.cube()
        self.cm        = pcsolver_v1p4.cm
        self.c.alloc_colors()
        hub.display.clear()
        self.portscan = True
        while self.portscan:
            time.sleep_ms(100)
            self.portscan = False
            print("B:Checking color sensor...")
            self.sensor_color = self.check_port(hub.port.B, False, [61],    4, 0)
            print("C:Checking distance sensor...")
            self.sensor_dist= self.check_port(hub.port.C, False, [62],    0, 2)
            print("D:Checking scanning motor...")
            self.motor_scan= self.check_port(hub.port.D, True,[48, 75], 4, 2)
            print("F:Checking turning motor...")
            self.motor_turn= self.check_port(hub.port.F, True,[48, 75], 4, 4)
            print("E:Checking tilting motor...")
            self.motor_tilt= self.check_port(hub.port.E, True,[48, 75], 0, 4)

    def check_port(self, port, motor, t, x, y):
        if motor:
            dev = port.motor
        else:
            dev = port.device
        if dev != None and (port.info()['type'] in t):
            hub.display.pixel(x, y, 0)
        else:
            if dev != None:
                print("Type: "+str(port.info()['type']))
            self.portscan = True
            hub.display.pixel(x, y, 9)
        return dev

    def Position(self, mot):
        return mot.get()[1]

    def run_nw(self, mot, pos, speed):
        mot.run_to_position(pos, speed, 75, mot.STOP_HOLD)

    def run_to(self, mot, pos, speed):
        mot.run_to_position(pos, speed, speed, mot.STOP_HOLD)
        while abs(self.Position(mot)-pos) > 3:
            time.sleep_ms(1)

    # MD: reduce speed: 35 -> 20, 25?
									 
    def ScanReset(self):
        self.ColorOff()
        for i in range(2):
            if i > 0:
                self.motor_scan.pwm(25)
                time.sleep_ms(100)
                self.motor_scan.brake()
                time.sleep_ms(100)
            self.motor_scan.pwm(-25)
            pos1 = self.Position(self.motor_scan)
            pos0 = pos1+100
            while pos1 < pos0:
                time.sleep_ms(100)
                pos0 = pos1
                pos1 = self.Position(self.motor_scan)
        self.motor_scan_base = self.Position(self.motor_scan)+10
        self.motor_scan.brake()

    def ScanPiece(self, pos, f, o, i):
        self.run_nw(self.motor_scan, self.motor_scan_base+pos, 100)
        self.Display(i)
        pos = self.motor_turn_base+self.turn_ratio*45
        self.motor_turn_base = pos
        pos -= self.turn_ratio*3
        while self.Position(self.motor_turn) < pos:
            time.sleep_ms(1)
        self.ScanRGB(f, o)
        if self.motor_scan.busy(1):
            self.slower = True

    def TurnReset(self):
        self.motor_turn_base = self.Position(self.motor_turn)
        self.motor_turn.brake()

    def TurnRotate(self, rot):
        self.motor_turn_base = self.motor_turn_base+self.turn_ratio*90*rot
        self.run_to(self.motor_turn, self.motor_turn_base, 80)

    def TurnTurn(self, rot, rotn):
        self.TiltHold()
        extra= self.turn_ratio*22
        extran = self.turn_ratio*3
        if rot < 0:
            extra = -extra
        if rotn < 0:
            extra -= extran
        elif rotn > 0:
            extra += extran
        self.motor_turn_base = self.motor_turn_base+self.turn_ratio*90*rot
        self.run_to(self.motor_turn, self.motor_turn_base+extra, 80)
        self.run_to(self.motor_turn, self.motor_turn_base, 80)

    def TiltReset(self):
        self.motor_tilt.pwm(40)
        pos1 = self.Position(self.motor_tilt)
        pos0 = pos1-100
        while pos1 > pos0:
            time.sleep_ms(100)
            pos0 = pos1
            pos1 = self.Position(self.motor_tilt)
        self.motor_tilt_base = self.Position(self.motor_tilt)-5
        self.motor_tilt.brake()

    def TiltAway(self, o=45):
        self.Eyes()
        self.run_nw(self.motor_tilt, self.motor_tilt_base, 40)
        pos = self.motor_tilt_base-o
        while self.Position(self.motor_tilt) < pos:
            time.sleep_ms(1)

    def TiltHold(self):
        self.run_to(self.motor_tilt, self.motor_tilt_base-75, 70)

    def TiltTilt(self):
        self.TiltHold()
        self.run_to(self.motor_tilt, self.motor_tilt_base-155, 70)
        time.sleep_ms(50)
        self.run_to(self.motor_tilt, self.motor_tilt_base-55, 100)
        self.run_nw(self.motor_tilt, self.motor_tilt_base-75, 70)
        time.sleep_ms(50)

    def ColorOff(self):
        self.sensor_color.mode(2)

    def ColorOn(self):
        self.sensor_color.mode(5)

    def CubeSense(self):
        cm = self.sensor_dist.get(self.sensor_dist.FORMAT_SI)[0]
        # print(cm)
        return cm != None and cm < 10

    def CubeRemove(self):
        self.Eyes()
        count = 0
        while count < 150:
            count += 1
            if self.CubeSense():
                count = 0
            time.sleep_ms(10)

    def CubeInsert(self):
        self.Eyes(0,0,3,3)
        count = hub.button.left.presses()+hub.button.right.presses()
        count = 0
        while count < 150:
            count += 1
            if not self.CubeSense():
                count = 0
            if hub.button.left.presses() > 0:
                # print("left")
                self.motor_turn_base -= 2*self.turn_ratio
                self.run_nw(self.motor_turn, self.motor_turn_base, 40)
            if hub.button.right.presses() > 0:
                # print("right")
                self.motor_turn_base += 2*self.turn_ratio
                self.run_nw(self.motor_turn, self.motor_turn_base, 40)
            time.sleep_ms(10)
        # MD: turn eyes off 
        #self.Eyes()

    def Init(self):
        self.motor_tilt.pwm(40)
        self.ScanReset()
        # MD: relax tension on the scanning arm
        self.run_to(self.motor_scan, self.motor_scan_base + 10, 100)
        self.TiltReset()
        self.TurnReset()

    def Eyes(self, a=0, b=0, c=0, d=0):
        self.sensor_dist.mode(5, b''+chr(a*9)+chr(b*9)+chr(c*9)+chr(d*9))

    def Show(self, s):
        hub.display.show(
            hub.Image('00000:0'+s[0:3]+'0:0'+s[3:6]+'0:0'+s[6:9]+'0:00000')
        )

    def Display(self, p):
        self.Show(('009000000', '000000009', '000000900',
                '900000000', '000009000', '000000090',
                '000900000', '090000000', '000090000')[p])

    def ScanRGB(self, f, o):
        rgb = self.sensor_color.get()
        self.c.set_rgb(f, o, rgb)
        rgb = ((2,0,0),
            (2,0,0),
            (2,1,0),
            (2,2,0),
            (0,2,0),
            (0,2,0),
            (0,0,2),
            (0,0,2),
            (2,2,2))[self.c.get_clr(f, o)]
        hub.led(rgb[0]*125, rgb[1]*20, rgb[2]*20)

    def ScanFace(self, f, o, tilt = True):
        if tilt:
            # MD: parking position? +100 -> +50
            self.run_nw(self.motor_scan, self.motor_scan_base+50, 100)
            # MD: or this is the parking position? +250 -> +70
            pos = self.motor_scan_base+70
            while self.Position(self.motor_scan) > pos:
                time.sleep_ms(1)
            self.TiltTilt()
        scanning = True
        while scanning:
            print("FACE "+str(f))
            self.TiltAway(5)
            self.Eyes(9,9,9,9)
            self.ColorOn()
            self.Display(8)

            # MD: middle piece: +485 -> +195
            self.run_to(self.motor_scan, self.motor_scan_base+195, 100)
            self.ScanRGB(f, 8)
            self.motor_tilt.brake()
            if self.slower:
                self.slower = False
                self.scan_speed -= 1
                # MD: decommented:
                print("Scan speed "+str(self.scan_speed))
            self.run_nw(self.motor_turn, self.motor_turn_base+self.turn_ratio*360, self.scan_speed)
            for i in range(4):
                # MD: corner piece: 300 -> 145
                self.ScanPiece(145, f, o, i)
                # MD: side piece: 365 -> 165
                self.ScanPiece(165, f, o+1, i+4)
                o += 2
                if o > 7:
                    o = 0
            scanning = self.slower
        self.ColorOff()
        hub.display.clear()

    def SolveCube(self):
        hub.led(0, 0, 0)
        hub.display.show(hub.Image.ARROW_SW)
        self.CubeInsert()
        hub.led(200, 15, 0)
        hub.display.show(hub.Image.DIAMOND)
        self.count += 1
        if self.count >= 10:
            self.count = 0
            self.ScanReset()
        scan = 0
        found = False
        while not found and scan < 3:
            scan += 1
            self.ScanFace(0, 4, False)
            self.ScanFace(4, 6)
            self.ScanFace(2, 0)
            self.TurnRotate(-1)
            self.ScanFace(3, 6)
            self.TurnRotate(1)
            self.ScanFace(5, 4)
            self.ScanFace(1, 4)
            self.Show('968776897')
            hub.led(200, 15, 0)
            t = -1
            for i in range(12):
                print("TYPE "+str(i))
                self.Eyes(5,5,5,5)
                c = self.c
                valid = c.determine_colors(i)
                # c.display()
                if valid:
                    t = i
                    print("Valid: "+str(t))
                    self.Eyes()
                    valid = c.valid_positions()
                    if valid:
                        found = True
                        break
            if not found and scan == 3 and t >= 0:
                found = c.determine_colors(t)
                # c.display()
                print("Invalid? "+str(t))
        # }
        if found:
            print("Solving...")
            self.Eyes(9,0,9)
            c.solve(2000)
            c.solve_apply()
            # c.display()
            self.Eyes(5,5)
            hub.led(0, 15, 10)
            self.Show('999999999')
            # MD: Parking? self.motor_scan_base -> self.motor_scan_base+50
            self.run_to(self.motor_scan, self.motor_scan_base+50, 100)
            c = self.c
            # Cube orientation after scan
            d = 3
            f = 2
            for mv in range(c.mv_n):
                md = c.mv_f[mv]
                mr = c.mv_r[mv]
                # print("Move ["+str(md)+" "+str(mr)+"]")
                # print("["+str(d)+" "+str(f)+"]")
                while d != md:
                    rm = self.cm.get_remap(d, f)
                    if md == rm.fm[2] or md == rm.fm[4]:
                        self.TiltTilt()
                        d = rm.fm[4]
                    elif md == rm.fm[5]:
                        self.TiltAway()
                        self.Eyes(5,5)
                        self.TurnRotate(2)
                        f = rm.fm[3]
                    elif md == rm.fm[3]:
                        self.TiltAway()
                        self.Eyes(5,5)
                        self.TurnRotate(1)
                        f = rm.fm[4]
                    else:
                        self.TiltAway()
                        self.Eyes(5,5)
                        self.TurnRotate(-1)
                        f = rm.fm[5]
                # }
                # print("["+str(d)+" "+str(f)+"]")
                mrn = 0
                mvn = mv+1
                while mvn < c.mv_n:
                    if self.cm.adjacent(c.mv_f[mvn], md):
                        mrn = c.mv_r[mvn]
                        break
                    mvn += 1
                # }
                self.TurnTurn(mr, mrn)
            # }
            hub.led(0, 50, 0)
            self.TiltAway()
            time.sleep_ms(500)
            self.TiltReset()
            self.Eyes(9,9,9,9)
            if c.mv_n > 0:
                self.TurnRotate(-6)
        # }
        else:
            print("Remove cube (MD)")
            # MD: Parking? self.motor_scan_base -> self.motor_scan_base+50
            self.run_to(self.motor_scan, self.motor_scan_base+50, 100)
            self.TiltReset()
        while (self.motor_scan.busy(1) or
            self.motor_turn.busy(1) or
            self.motor_tilt.busy(1)):
            time.sleep_ms(1)
        self.motor_scan.brake()
        self.motor_turn.brake()
        self.motor_tilt.brake()
        hub.display.show(hub.Image.ARROW_NE)
        self.CubeRemove()

#-----------------------------------------------------------------------------

def main():
    print("main()")
    pc = primecuber()
    print("Init()")
    pc.Init()
    while True:
        print("SolveCube()")
        pc.SolveCube()

trace("imported")

#-----------------------------------------------------------------------------

# END
