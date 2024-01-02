#-----------------------------------------------------------------------------
# Title:        PrimeCuber Solver
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
# Purpose:    Rubik's Cube solver for PrimeCuber robot
#-----------------------------------------------------------------------------

import gc, os, time

gc.collect()

def trace(msg):
    if False:
        gc.collect()
        print("TRACE: "+msg+" mem="+str(gc.mem_free()))

trace("module pccolors_v1p4")

CMAX = 1024

trace("class color")
class color():

    def __init__(self):
        self.set_rgb(0, 0, 0)

    def set_rgb(self, r, g, b):
        # Convert to hsl
        h= 0
        s= 0
        sl = 0
        l= 0
        v= r
        if g > v:
            v = g
        if b > v:
            v = b
        m = r
        if g < m:
            m = g
        if b < m:
            m = b
        vf = v+m
        l = int(vf/2)
        if l > 0:
            vm = v-m
            if vm > 0:
                if vf <= CMAX:
                    vf = 2*CMAX-vf
                s = int(CMAX*vm/vf)
                if r == v:
                    h = 0*CMAX+int(CMAX*(g-b)/vm)
                elif g == v:
                    h = 2*CMAX+int(CMAX*(b-r)/vm)
                else:
                    h = 4*CMAX+int(CMAX*(r-g)/vm)
            h += CMAX # rotate so R/B either side of 0
            h = int(h/6)
            if h < 0:
                h += CMAX
            elif h >= CMAX:
                h -= CMAX
            # Emphasize low saturation for bright colors (e.g. white)
            sl = int(CMAX*s/l)
        # }
        self.r= r
        self.g= g
        self.b= b
        self.h= h
        self.sl = sl
        self.l= l

#-----------------------------------------------------------------------------

NFACE = 6

def POS(f, o):
    return f*9+o

trace("class cube_colors")
class cube_colors():

    def __init__(self, cb):
        self.cb= cb
        self.clrs = []
        for i in range(NFACE*9):
            self.clrs.append(color())

    def set_col(self, f, o, c):
        self.cb.pce[f][o] = c

    def clr_ratio(self, c0, c1):
        ratio = 0
        if c0 < c1:
            ratio = -int(2000*(c1-c0)/(c1+c0))
        elif c0 > c1:
            ratio =int(2000*(c0-c1)/(c1+c0))
        return ratio

    def cmp_h(self, c0, c1):
        return c1.h> c0.h

    def cmp_sl(self, c0, c1):
        return c1.sl > c0.sl

    def cmp_slr(self, c0, c1):
        return c1.sl < c0.sl

    def cmp_l(self, c0, c1):
        return c1.l> c0.l

    def cmp_lr(self, c0, c1):
        return c1.l< c0.l

    def cmp_r_g(self, c0, c1):
        return self.clr_ratio(c1.r, c1.g) < self.clr_ratio(c0.r, c0.g)

    def cmp_r_b(self, c0, c1):
        return self.clr_ratio(c1.r, c1.b) < self.clr_ratio(c0.r, c0.b)

    def cmp_b_g(self, c0, c1):
        return self.clr_ratio(c1.b, c1.g) < self.clr_ratio(c0.b, c0.g)

    def sort_clrs(self, co, b, n, cmp_fn):
        e= b+n-2
        ib = b
        ie = e
        while (ib <= ie):
            il = e+2
            ih = b-2
            i= ib
            while i <= ie:
                if cmp_fn(self.clrs[co[i+1]], self.clrs[co[i]]):
                    o    = co[i]
                    co[i] = co[i+1]
                    co[i+1] = o
                    if i < il:
                        il = i
                    if i > ih:
                        ih = i
                # }
                i += 1
            # }
            ib = il-1
            if ib < b:
                ib = b
            ie = ih+1
            if ie > e:
                ie = e
        # }

    def sort_colors(self, co, t, s):
        if t < 6:
            # Lightness
            self.sort_clrs(co, 0, 6*s, self.cmp_lr)
            # Saturation
            self.sort_clrs(co, 0, 3*s, self.cmp_sl)
        else:
            # Saturation
            self.sort_clrs(co, 0, 6*s, self.cmp_sl)
        # }
        # Hue
        self.sort_clrs(co, s, 5*s, self.cmp_h)
        # Red/Orange
        cmp_fn = (None,
                self.cmp_r_g,
                self.cmp_b_g,
                self.cmp_r_b,
                self.cmp_slr,
                self.cmp_l)[t % 6]
        if cmp_fn != None:
            self.sort_clrs(co, s, 2*s, cmp_fn)
        i = 0
        while i < 1*s:
            self.clrs[co[i]].clr = 0
            i += 1
        while i < 2*s:
            self.clrs[co[i]].clr = 4
            i += 1
        while i < 3*s:
            self.clrs[co[i]].clr = 5
            i += 1
        while i < 4*s:
            self.clrs[co[i]].clr = 2
            i += 1
        while i < 5*s:
            self.clrs[co[i]].clr = 1
            i += 1
        while i < 6*s:
            self.clrs[co[i]].clr = 3
            i += 1

    def determine_colors(self, t):
        clr_ord = [0] * (NFACE*4)
        for i in range(NFACE):
            clr_ord[i] = POS(i, 8)
        self.sort_colors(clr_ord, t, 1)
        for i in range(NFACE):
            clr_ord[4*i+0] = POS(i, 0)
            clr_ord[4*i+1] = POS(i, 2)
            clr_ord[4*i+2] = POS(i, 4)
            clr_ord[4*i+3] = POS(i, 6)
        # }
        self.sort_colors(clr_ord, t, 4)
        for i in range(NFACE):
            clr_ord[4*i+0] = POS(i, 1)
            clr_ord[4*i+1] = POS(i, 3)
            clr_ord[4*i+2] = POS(i, 5)
            clr_ord[4*i+3] = POS(i, 7)
        # }
        self.sort_colors(clr_ord, t, 4)
        clr_map = [0] * NFACE
        for f in range(NFACE):
            clr_map[self.clrs[POS(f, 8)].clr] = f
        for f in range(NFACE):
            for o in range(8):
                self.set_col(f, o, clr_map[self.clrs[POS(f, o)].clr])
        # }
        return self.cb.valid_pieces()

    def set_rgb(self, f, o, rgb):
        self.clrs[POS(f, o)].set_rgb(rgb[0], rgb[1], rgb[2])

    def get_clr(self, f, o):
        clr = self.clrs[POS(f, o)]
        c = 8 # white
        if clr.sl > 50:
            c = int(8*clr.h/CMAX)
        return c

#    def str3(self, s):
#        return (""+str(s))[-3:]
#
#    def hsl(self, f, o):
#        c = self.clrs[POS(f,o)]
#        if o == 8:
#            p = f;
#        else:
#            p = c.pce[f][o];
#        return "["+str(p)+":"+self.str3(c.r)+" "+self.str3(c.g)+" "+self.str3(c.b)+":"+self.str3(c.h)+" "+self.str3(c.sl)+" "+self.str3(c.l)+"]"
#
#    def display_line(self, f, l):
#        if l == 0:
#            s = self.hsl(f,2)+" "+self.hsl(f,3)+" "+self.hsl(f,4)
#        elif l == 1:
#            s = self.hsl(f,1)+" "+self.hsl(f,8)+" "+self.hsl(f,5)
#        else:
#            s = self.hsl(f,0)+" "+self.hsl(f,7)+" "+self.hsl(f,6)
#        return s
#
#    def display(self):
#        for l in range(3):
#            print((" "*84)+self.display_line(4, l))
#        for l in range(3):
#            print(self.display_line(0, l)+" "+
#                self.display_line(1, l)+" "+
#                self.display_line(2, l)+" "+
#                self.display_line(3, l))
#        for l in range(3):
#            print((" "*84)+self.display_line(5, l))

trace("imported")

#-----------------------------------------------------------------------------

# END
