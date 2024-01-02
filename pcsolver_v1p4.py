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

trace("module pcsolver_v1p4")

NFACE    = 6
NSIDE    = 4
NSIDE_M1 = NSIDE-1
NCORNER= int(NFACE*NSIDE/3)
NEDGE    = int(NFACE*NSIDE/2)

def RMOD(r):
    return r & NSIDE_M1

def RFIX(r):
    return ((r+1) & NSIDE_M1)-1

def POS(f, o):
    return f*9+o

#-----------------------------------------------------------------------------

trace("class remap")
class remap():

    def __init__(self):
        self.fm = [-1] * NFACE
        self.rm = [-1] * NFACE

    def init_maps(self, f, r):
        self.fm[f] = r
        self.rm[r] = f

#-----------------------------------------------------------------------------

trace("class face_map")
class face_map():

    def __init__(self):
        self.face        = -1
        self.face_edge= [-1] * NFACE
        self.face_corner = [-1] * NFACE

    def init(self, f, f0, f1, f2, f3):
        self.face = f
        self.fce= [f0, f1, f2, f3]

    def init_rest(self):
        for d in range(NSIDE):
            f = self.fce[d].face
            self.face_edge[f]= 2*d+1
            self.face_corner[f] = (self.face_edge[f]+1)&(2*NSIDE-1)

    def dir1(self, d):
        return self.fce[d].face

    def dir(self, f, d):
        fd = -1
        for i in range(NSIDE):
            if self.fce[i].face == f:
                m = self.fce[i]
                for j in range(NSIDE):
                    if m.fce[j].face == self.face:
                        fd = m.fce[RMOD(j+d)].face
                        break
                break
        return fd

#-----------------------------------------------------------------------------

trace("class cube_map")
class cube_map():

    def __init__(self):
        self.map = []
        self.rm= []
        self.dst = []
        for i in range(NFACE):
            self.map.append(face_map())
            self.rm.append([])
            self.dst.append([-1] * NFACE)
            for j in range(NFACE):
                self.rm[i].append(remap())
        self.map[0].init(0, self.map[3], self.map[4], self.map[1], self.map[5])
        self.map[1].init(1, self.map[0], self.map[4], self.map[2], self.map[5])
        self.map[2].init(2, self.map[1], self.map[4], self.map[3], self.map[5])
        self.map[3].init(3, self.map[2], self.map[4], self.map[0], self.map[5])
        self.map[4].init(4, self.map[0], self.map[3], self.map[2], self.map[1])
        self.map[5].init(5, self.map[0], self.map[1], self.map[2], self.map[3])
        for i in range(NFACE):
            self.map[i].init_rest()
        for f0 in range(NFACE):
            for f1 in range(NFACE):
                self.dst[f0][f1] = 2
        ff0 = 0
        for fr0 in range(NFACE):
            self.dst[fr0][fr0] = 0
            for d0 in range(NSIDE):
                ff1 = 1
                fr1 = self.map[fr0].dir1(d0)
                self.dst[fr0][fr1] = 1
                for d1 in range(NSIDE):
                    ff2 = self.map[ff0].dir(ff1, d1)
                    fr2 = self.map[fr0].dir(fr1, d1)
                    for d2 in range(NSIDE):
                        ff3 = self.map[ff1].dir(ff2, d2)
                        fr3 = self.map[fr1].dir(fr2, d2)
                        self.rm[fr0][fr1].init_maps(ff3, fr3)

    def dir(self, f, d):
        return self.map[f].dir1(d)

    def adjacent(self, f0, f1):
        return self.dst[f0][f1] == 1

    def edge(self, f0, f1):
        return self.map[f0].face_edge[f1]

    def corner(self, f0, f1):
        return self.map[f0].face_corner[f1]

    def get_remap(self, f0, f1):
        return self.rm[f0][f1]

#-----------------------------------------------------------------------------

# Default to small tables included in this file
large = False

# Use large tables if they have been downloaded
try:
    large = os.stat("/pcmtab4_v1p4.bin")[6] == 2561877
except:
    None

if large:
    # Large tables
    print("Using large table: pcmtab4_v1p4.bin")
    trace("class cube_mtab4")
    class cube_mtab4:

        NSTAGE = 4    # Number of stages in solve
        NPIECE = 4    # Maximum number of corners/edges per stage

        def init(c):
            # 0
            s = c.stage(0)
            c.adde(s, 0, 1)
            c.adde(s, 0, 4)
            c.adde(s, 1, 4)
            c.addc(s, 0, 4, 1)
            c.send(s)

            # 1
            s = c.stage(s)
            c.adde(s, 0, 3)
            c.adde(s, 0, 5)
            c.adde(s, 5, 1)
            c.addc(s, 0, 1, 5)
            c.send(s)

            # 2
            s = c.stage(s)
            c.adde(s, 3, 5)
            c.adde(s, 4, 3)
            c.addc(s, 0, 3, 4)
            c.addc(s, 0, 5, 3)
            c.send(s)

            # 3
            s = c.stage(s)
            c.adde(s, 2, 1)
            c.adde(s, 2, 3)
            c.adde(s, 2, 4)
            c.addc(s, 2, 1, 4)
            c.addc(s, 2, 3, 5)
            c.addc(s, 2, 4, 3)
            c.send(s)

            # Unused stage since last corner and edge will already be solved
            s = c.stage(s)
            c.adde(s, 2, 5)
            c.addc(s, 2, 5, 1)

        file_name = "/pcmtab4_v1p4.bin"

        mtb = (
            5, 6, 7, 9
            )

        MV_MENT = 17

    cube_mtab = cube_mtab4

else:
    # Mini tables

    trace("class cube_mtab1")
    class cube_mtab1:

        NSTAGE = 8    # Number of stages in solve
        NPIECE = 3    # Maximum number of corners/edges per stage

        def init(c):
            # 0
            s = c.stage(0)
            c.adde(s, 2, 1)
            c.adde(s, 2, 4)
            c.send(s)

            # 1
            s = c.stage(s)
            c.adde(s, 1, 4)
            c.addc(s, 2, 1, 4)
            c.send(s)

            # 2
            s = c.stage(s)
            c.adde(s, 2, 3)
            c.adde(s, 2, 5)
            c.send(s)

            # 3
            s = c.stage(s)
            c.adde(s, 4, 3)
            c.addc(s, 2, 4, 3)
            c.send(s)

            # 4
            s = c.stage(s)
            c.adde(s, 3, 5)
            c.addc(s, 2, 3, 5)
            c.send(s)

            # 5
            s = c.stage(s)
            c.adde(s, 0, 3)
            c.adde(s, 0, 4)
            c.addc(s, 0, 3, 4)
            c.send(s)

            # 6
            s = c.stage(s)
            c.addc(s, 0, 1, 5)
            c.addc(s, 0, 4, 1)
            c.addc(s, 2, 5, 1)
            c.send(s)

            # 7
            s = c.stage(s)
            c.adde(s, 0, 1)
            c.adde(s, 5, 1)
            c.send(s)

            # Unused stage since last corner and edge will already be solved
            s = c.stage(s)
            c.adde(s, 0, 5)
            c.addc(s, 0, 5, 3)

        file_name = "/pcmtab1_v1p4.bin"

        mtb = (
            3, 4, 4, 5, 5, 6, 7, 7
            )

        MV_MENT = 13

    cube_mtab = cube_mtab1

trace("class solve_map")
class solve_map():

    NSTAGE = cube_mtab.NSTAGE
    NPIECE = cube_mtab.NPIECE

    def __init__(self):
        # Offset into cp/ep tables for each stage
        self.cn    = [-1] * (solve_map.NSTAGE+2)
        self.en    = [-1] * (solve_map.NSTAGE+2)
        self.sz    = [-1] * solve_map.NSTAGE

        # Unrotated corner and edge positions - reverse solve order
        self.cp0= [-1] * NCORNER
        self.cp1= [-1] * NCORNER
        self.cp2= [-1] * NCORNER
        self.ep0= [-1] * NEDGE
        self.ep1= [-1] * NEDGE

        self.cn[0] = NCORNER
        self.en[0] = NEDGE

        cube_mtab.init(self)

    def addc(self, s, c0, c1, c2):
        i = self.cn[s]-1
        self.cn[s]= i
        self.cp0[i] = c0
        self.cp1[i] = c1
        self.cp2[i] = c2

    def adde(self, s, e0, e1):
        i = self.en[s]-1
        self.en[s]= i
        self.ep0[i] = e0
        self.ep1[i] = e1

    def stage(self, s):
        self.cn[s+1] = self.cn[s]
        self.en[s+1] = self.en[s]
        return s+1

    def send(self, s):
        msg = "STAGE: "+str((s-1))
        idx = 1
        nc= self.cn[s-1]-self.cn[s]
        msg += " C"+str(nc)
        if nc > 0:
            i = self.cn[s]
            while (i < self.cn[s-1]):
                msg += " ["+str(self.cp0[i])+","+str(self.cp1[i])+","+str(self.cp2[i])+"]"
                idx *= 3*(i+1)
                i += 1
        ne = self.en[s-1]-self.en[s]
        msg += " E"+str(ne)
        if ne > 0:
            i = self.en[s]
            while (i < self.en[s-1]):
                msg += " ["+str(self.ep0[i])+","+str(self.ep1[i])+"]"
                idx *= 2*(i+1)
                i += 1
        if s == solve_map.NSTAGE:
            idx = int(idx/2)
        self.sz[s-1] = idx
        print(msg+" SZ="+str(self.sz[s-1]))

#-----------------------------------------------------------------------------

trace("class mtab")
class mtab():

    def __init__(self, s):
        self.stage= s
        self.sz    = sm.sz[s]
        self.nbytes = cube_mtab.mtb[s]
        self.foff= 0
        for i in range(s):
            self.foff += (sm.sz[i]-1)*cube_mtab.mtb[i]
        self.fmap= (0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 4, 5, 5, 5)
        self.rmap= (1, 2, -1) * NFACE

    def moves(self, i, f, r):
        mv = 0
        if i > 0:
            d = (i-1)*self.nbytes
            # File based data
            # print("Read stage="+str(s)+
            #    " file="+cube_mtab.file_name+
            #    " offset="+str(self.foff)+
            #    "+"+str(d)+
            #    " bytes="+str(self.nbytes))
            fs = open(cube_mtab.file_name, 'rb')
            fs.seek(self.foff+d)
            data = fs.read(self.nbytes)
            # print(data)
            fs.close()
            d = 0
            b = data[d]
            d += 1
            if b != 0xFF:
                mvm = self.nbytes*2-1
                f0 = self.fmap[b]
                f[mv] = f0
                r[mv] = self.rmap[b]
                mv += 1
                while (mv < mvm):
                    b >>= 4
                    if (mv & 1) != 0:
                        b = data[d]
                        d += 1
                    b0 = b & 0xF
                    if b0 == 0xF:
                        break
                    f0 = self.fmap[b0]
                    r[mv] = self.rmap[b0]
                    if f0 >= f[mv-1]:
                        f0 += 1
                    f[mv] = f0
                    mv += 1

        return mv

#-----------------------------------------------------------------------------

trace("class cube_idx")
class cube_idx():

    def __init__(self, m = None):
        self.ci = []
        self.ei = []
        for i in range(NFACE):
            self.ci.append([0] * NFACE)
            self.ei.append([0] * NFACE)
        # pre-allocate for speed
        self.tmp_idx = [0] * solve_map.NPIECE

        self.init(m)

    def init(self, m):
        if m != None:
            for i in range(NCORNER):
                cp0 = sm.cp0[i]
                cp1 = sm.cp1[i]
                cp2 = sm.cp2[i]
                c0 = m.corner(cp0, cp1)
                c1 = m.corner(cp1, cp2)
                c2 = m.corner(cp2, cp0)
                i3 = 3*i
                self.ci[c0][c1] = i3+2
                self.ci[c1][c2] = i3+1
                self.ci[c2][c0] = i3
            # }
            for i in range(NEDGE):
                ep0 = sm.ep0[i]
                ep1 = sm.ep1[i]
                e0 = m.edge(ep0, ep1)
                e1 = m.edge(ep1, ep0)
                i2 = 2*i
                self.ei[e0][e1] = i2+1
                self.ei[e1][e0] = i2
            # }

    def index(self, s):
        idx = self.tmp_idx
        ind = 0
        cs = sm.cn[s]
        ce = sm.cn[s+1]
        cn = cs - ce
        cm = 3*cs
        for i in range(cn):
            cp0 = sm.cp0[cs-i-1]
            cp1 = sm.cp1[cs-i-1]
            ii = self.ci[cp0][cp1]
            for j in range(i):
                if ii > idx[j]:
                    ii -= 3
            # }
            idx[i] = ii
            ind = (ind*cm)+ii
            cm -= 3
        # }
        es = sm.en[s]
        ee = sm.en[s+1]
        en = es - ee
        em = 2*es
        for i in range(en):
            ep0 = sm.ep0[es-i-1]
            ep1 = sm.ep1[es-i-1]
            ii = self.ei[ep0][ep1]
            for j in range(i):
                if ii > idx[j]:
                    ii -= 2
            # }
            idx[i] = ii
            ind = (ind*em)+ii
            em -= 2
        # }
        if s == (solve_map.NSTAGE-1):
            # Minimise index when parity known
            if en > 0:
                ind = (int(ind/4)*2)+(ind&1)
            else:
                ind = (int(ind/6)*3)+(ind%3)
        # }
        sz = sm.sz[s]
        ind = (sz-1-ind)
        return ind

#-----------------------------------------------------------------------------

MAXINT = 0x7FFFFFFF

MV_MAX = 80

import random

def RND(max):
    return random.getrandbits(19) % max

trace("class cube")
class cube():

    colors = None

    def __init__(self):
        self.mv_n    = 0
        self.found    = 0
        self.quick    = 0
        self.end_time = 0
        self.pce    = [0] * NFACE
        for f in range(NFACE):
            self.pce[f] = [f] * (2*NSIDE)
        self.mv_f    = [0] * MV_MAX
        self.mv_r    = [0] * MV_MAX
        self.colors= None
        # pre-allocate for speed
        self.tmp_f    = [0] * cube_mtab.MV_MENT
        self.tmp_r    = [0] * cube_mtab.MV_MENT
        self.tmp_mi= cube_idx()

    def alloc_colors(self):
        if self.colors == None:
            self.colors= colors.cube_colors(self)

    def copy(self, m):
        for f in range(NFACE):
            for i in range(2*NSIDE):
                self.pce[f][i] = m.pce[f][i]

    def copy_moves(self, m):
        self.mv_n = m.mv_n
        for i in range(self.mv_n):
            self.mv_f[i] = m.mv_f[i]
            self.mv_r[i] = m.mv_r[i]

    def corner(self, f0, f1):
        return self.pce[f0][cm.corner(f0, f1)]

    def edge(self, f0, f1):
        return self.pce[f0][cm.edge(f0, f1)]

    def rot(self, f, r):
        r= RMOD(r)
        p0 = self.pce[f]
        fd = cm.dir(f, NSIDE_M1)
        while r > 0:
            r-= 1
            p= p0[6]; p0[6] = p0[4]; p0[4] = p0[2]; p0[2] = p0[0]; p0[0] = p
            p= p0[7]; p0[7] = p0[5]; p0[5] = p0[3]; p0[3] = p0[1]; p0[1] = p
            pd= self.pce[fd]
            od2 = cm.corner(fd, f)
            od0 = (od2-2)&7
            c0= pd[od0]
            e0= pd[od0+1]
            c1= pd[od2]
            for d in range(NSIDE-2, -1, -1):
                fs= cm.dir(f, d)
                os2 = cm.corner(fs, f)
                os0 = (os2-2)&7
                ps= self.pce[fs]
                pd[od0]= ps[os0]
                pd[od0+1] = ps[os0+1]
                pd[od2]= ps[os2]
                pd= ps
                od0 = os0
                od2 = os2
            # }
            pd[od0]= c0
            pd[od0+1] = e0
            pd[od2]= c1

    def backtrack_a(self, f):
        i = self.mv_n
        btrack = False
        while (i > 0):
            i -= 1
            fi = self.mv_f[i]
            if cm.adjacent(f, fi):
                break
            if f <= fi:
                btrack = True
                break
        # }
        return btrack

    def add_mv(self, f, r):
        i = self.mv_n
        mrg = False
        while (i > 0):
            i -= 1
            fi = self.mv_f[i]
            if cm.adjacent(f, fi):
                break
            if f == fi:
                r += self.mv_r[i]
                r = RFIX(r)
                if r != 0:
                    self.mv_r[i] = r
                else:
                    self.mv_n -= 1
                    while (i < self.mv_n):
                        self.mv_f[i] = self.mv_f[i+1]
                        self.mv_r[i] = self.mv_r[i+1]
                        i += 1
                    # }
                # }
                mrg = True
                break
            # }
        # }
        if not mrg:
            self.mv_f[self.mv_n] = f
            self.mv_r[self.mv_n] = RFIX(r)
            self.mv_n += 1
        # }

    def move(self, f, r):
        self.rot(f, r)
        self.add_mv(f, r)

    def valid_pieces(self):
        val = True
        for f0 in range(NFACE):
            for f1 in range(NFACE):
                if cm.adjacent(f0, f1):
                    efound = False
                    cfound = False
                    f2 = cm.get_remap(f0, f1).fm[5]
                    for s0 in range(NFACE):
                        for s1 in range(NFACE):
                            if cm.adjacent(s0, s1):
                                s2 = cm.get_remap(s0, s1).fm[5]
                                if (self.edge(s0, s1) == f0 and
                                    self.edge(s1, s0) == f1):
                                    efound = True
                                if (self.corner(s0, s1) == f0 and
                                    self.corner(s1, s2) == f1 and
                                    self.corner(s2, s0) == f2):
                                    cfound = True
                        # }
                    # }
                    if not (efound and cfound):
                        val = False
        return val

    def valid_positions(self):
        c = cube()
        c.copy(self)
        c.solve(0)
        c.solve_apply()
        return c.solved()

    def solved(self):
        slvd = True
        f = 0
        while (slvd and f < NFACE):
            for p in range(2*NSIDE):
                if self.pce[f][p] != f:
                    slvd = False
                    break
            # }
            f += 1
        # }
        return slvd

    def shuffle(self, n):
        for i in range(n):
            self.rot(RND(NFACE), 1+RND(NSIDE_M1))

    def timeout(self):
        return time.ticks_ms() >= self.end_time

    def solve_remap(self, best, s0):
        slvd = True
        for s in range(s0, solve_map.NSTAGE):
            mi = self.tmp_mi
            mi.init(self)
            i = mi.index(s)
            if i != 0:
                n = -1
                f = self.tmp_f
                r = self.tmp_r
                n = mt[s].moves(i, f, r)
                if n > 0:
                    mv = self.mv_n + n
                    for j in range(n):
                        self.add_mv(f[j], r[j])
                    # }
                    if (self.mv_n > best or
                        (s == 0 and self.mv_n < mv)):
                        slvd = False
                        break
                    # }
                    if s < (solve_map.NSTAGE-1):
                        for j in range(n):
                            self.rot(f[j], r[j])
                    # }
                else:
                    slvd = False
                    break
                # }
            # }
        # }
        return slvd

    def solve_one(self, cb, cs, depth):
        slvd = False
        if self.mv_n < depth:
            f = 0
            while (not slvd and f < NFACE):
                if not self.backtrack_a(f):
                    n = self.mv_n+1
                    for i in range(1, NSIDE):
                        self.move(f, 1)
                        if not slvd and self.solve_one(cb, cs, depth):
                            slvd = True
                    # }
                    self.move(f, 1)
                # }
                f += 1
            # }
        else:
            # print("solve_one: quick="+str(cb.quick))
            cs.copy(self)
            cs.copy_moves(self)
            cs.end_time = cb.end_time
            if cs.solve_remap(cb.mv_n, 0):
                if cs.mv_n < cb.mv_n:
                    # print("solve_one: solved="+str(cs.mv_n))
                    cb.copy_moves(cs)
                # }
            # }
            # finish if a short solution has been found or if any solution
            # has been found and the timeout has expired or if only a quick
            # solve is required
            if cb.mv_n <= 8 or (cb.mv_n <MV_MAX and (cb.quick or cb.timeout())):
                trace("solved_one()")
                slvd = True
        return slvd

    def solve(self, msecs = 1000):
        start_time = time.ticks_ms()
        self.mv_n = MAXINT
        self.end_time = start_time + msecs
        self.quick = (msecs == 0)
        cw = cube()
        cw.copy(self)
        cs = cube()
        depth = 0
        while (not cw.solve_one(self, cs, depth)):
            depth += 1
        print("Moves: "+str(self.mv_n)+" "+"Time: "+str(int(time.ticks_ms() - start_time))+"ms ")

    def solve_apply(self):
        for i in range(self.mv_n):
            self.rot(self.mv_f[i], self.mv_r[i])

    def set_rgb(self, f, o, rgb):
        self.colors.set_rgb(f, o, rgb)

    def get_clr(self, f, o):
        return self.colors.get_clr(f, o)

    def determine_colors(self, t):
        return self.colors.determine_colors(t)

def init(colors_module):
    trace("init()")
    global colors, cm, sm, mt
    colors = colors_module
    cm    = cube_map()
    sm    = solve_map()
    mt    = []
    for s in range(cube_mtab.NSTAGE):
        mt.append(mtab(s))
    trace("done")

trace("imported")

#-----------------------------------------------------------------------------

# END
