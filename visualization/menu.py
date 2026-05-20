import pygame
import os
from datetime import datetime
import math

BG        = (10,  16,  28)
PANEL_BG  = (18,  28,  48)
PANEL_BRD = (38,  68, 115)
PREV_BG   = (14,  22,  40)
TITLE_C   = (100, 195, 255)
SECT_C    = (70,  145, 225)
TEXT_C    = (195, 220, 255)
DIM_C     = (90,  120, 170)
BTN_NRM   = (28,  48,  86)
BTN_HVR   = (48,  78, 144)
BTN_SEL   = (18, 108, 202)
SEL_BRD   = (80, 165, 255)
NRM_BRD   = (50,  80, 135)
SEL_SUB   = (150, 185, 255)
DIM_SUB   = (55,  80, 125)
GREEN_N   = (20, 130,  60)
GREEN_H   = (35, 170,  82)
ORANGE_N  = (165, 82,  18)
ORANGE_H  = (210, 112, 38)
BLUE_N    = (18,  90, 175)
BLUE_H    = (28, 120, 210)
GRID_C    = (20,  32,  56)
SEP_C     = (35,  58, 100)

ROBOT_INFO = {
    "differential": {"title":"Diferansiyel Suruclu","props":["2 tahrik tekerlegi","Non-holonomik","ICR tabanli donus","Palet / tekerlekli"],"color":(40,80,170),"track":(55,60,75),"holo":False},
    "omni":         {"title":"Omniwheel (3-Tekerlek)","props":["3 x 120 derece tekerlek","Holonomik","Ani yon degisimi","Lab / arastirma"],"color":(0,155,175),"track":None,"holo":True},
    "ackermann":    {"title":"Ackermann Direksiyon","props":["On tekerlek donusu","Non-holonomik","Min. donus yari capi","Otonom arac tabanli"],"color":(175,88,10),"track":None,"holo":False},
    "mecanum":      {"title":"Mecanum Tekerlek","props":["4 x mecanum tekerlek","Holonomik","Yatay kayma hareketi","Depo / lojistik"],"color":(105,28,155),"track":(70,50,90),"holo":True},
    "drone":        {"title":"Kuadrotor Drone","props":["4 x rotor (X-yapilanma)","Holonomik","3D manevra kabiliyeti","Hava arastirma / gozetleme"],"color":(20,160,60),"track":None,"holo":True},
    "vtol":         {"title":"VTOL Hava Araci","props":["Dikey inis-kalkis","Holonomik (hover)","Sabit kanat gecisi","Askeri / kargo"],"color":(0,140,200),"track":None,"holo":True},
    "fixedwing":    {"title":"Sabit Kanatli Ucak","props":["Kanat kaldiricisi ile ucus","Non-holonomik","Min. donus yari capi","Uzun menzil / kesfiye"],"color":(180,30,30),"track":None,"holo":False},
    "quadruped":    {"title":"Quadruped - Robot Kopek","props":["4 bacakli yuruyus gayt","Holonomik benzeri","Yanal hiz kisitli (x0.60)","Boston Dynamics Spot"],"color":(30,140,80),"track":None,"holo":False},
    "hexapod":      {"title":"Hexapod - 6 Bacakli Robot","props":["Tripod gayt sistemi","Gercek holonomik","Genis tabana kararlilik","Arastirma / kurtarma"],"color":(160,110,20),"track":None,"holo":True},
    "snake":        {"title":"Yilan Robot - N-Eklemli","props":["N=6 govde segmenti","Non-holonomik","Dar gecit uzmani","Enkaz arama / endoskopi"],"color":(20,145,155),"track":None,"holo":False},
    "bipedal":      {"title":"Bipedal - Insansi Robot","props":["2 bacakli adim modeli","Non-holonomik","Donus sirasinda hiz dusuyor","Atlas / ASIMO benzeri"],"color":(145,40,130),"track":None,"holo":False},
}

# 3 kategori halinde sıralanmış robot seçenekleri
ROBOT_ROWS = [
    ("Tekerlek", [
        ("differential","Diferansiyel","Non-holo / Palet"),
        ("omni",        "Omniwheel",   "Holo / 3-teker"),
        ("ackermann",   "Ackermann",   "Non-holo / Araba"),
        ("mecanum",     "Mecanum",     "Holo / 4-mecanum"),
    ]),
    ("Bacakli Robot", [
        ("quadruped","Quadruped", "Robot Kopek"),
        ("hexapod",  "Hexapod",  "6-Bacak"),
        ("snake",    "Yilan",    "N-Eklemli"),
        ("bipedal",  "Bipedal",  "Insansi"),
    ]),
    ("Hava Araci", [
        ("drone",     "Drone",       "Holo / Kuadrotor"),
        ("vtol",      "VTOL",        "Holo / Hover"),
        ("fixedwing", "Sabit Kanat", "Non-holo / Ucak"),
    ]),
]
# Geriye dönük uyumluluk için düz liste
ROBOT_OPTS = [item for _,row in ROBOT_ROWS for item in row]
GLOBAL_PLANNER_OPTS = [
    ("astar",   "A*",       "Buluristik sezgisel"),
    ("dijkstra","Dijkstra", "Garantili en kisa"),
    ("dstar",   "D* Lite",  "Dinamik replanlama"),
    ("rrt",     "RRT",      "Ornekleme tabanli"),
    ("rrtstar", "RRT*",     "Asimptotik optimal"),
]
LOCAL_PLANNER_OPTS = [
    ("bug0",            "Bug0",       "Reaktif / basit"),
    ("bug1",            "Bug1",       "Tam cevreleme"),
    ("bug2",            "Bug2",       "M-line tabanli"),
    ("potentialfields", "Pot.Fields", "Kuvvet alani"),
    ("vfh",             "VFH",        "Vadi histogrami"),
]
LOCAL_PLANNER_KEYS = {k for k, _, _ in LOCAL_PLANNER_OPTS}
# Geriye dönük uyumluluk için
PLANNER_OPTS = GLOBAL_PLANNER_OPTS
METRIC_OPTS = [
    ("euclidean", "Oklid",    "sqrt(dx2+dy2)"),
    ("manhattan", "Manhattan","|dx|+|dy|"),
    ("chebyshev", "Chebyshev","max(|dx|,|dy|)"),
    ("octile",    "Octile",   "Karma"),
    ("diagonal",  "Diagonal", "Diagonal"),
    ("minkowski2","Mink-2",   "p=2"),
    ("minkowski3","Mink-3",   "p=3"),
]
CRITERIA_OPTS = [
    ("shortest", "En Kisa",   "Min. yol uzunlugu"),
    ("safest",   "En Guvenli","Max. engel mesafesi"),
    ("fastest",  "En Hizli",  "Min. gecis suresi"),
    ("smoothest","En Duzgun", "Min. donus sayisi"),
]


class Button:
    def __init__(self,rect,label,sublabel="",tag=None,font_lbl=None,font_sub=None):
        self.rect=pygame.Rect(rect);self.label=label;self.sublabel=sublabel
        self.tag=tag;self.font_lbl=font_lbl;self.font_sub=font_sub
        self.selected=False;self.hovered=False
    def draw(self,surf):
        col=BTN_SEL if self.selected else(BTN_HVR if self.hovered else BTN_NRM)
        brd=SEL_BRD if(self.selected or self.hovered)else NRM_BRD
        bw=2 if self.selected else 1
        pygame.draw.rect(surf,col,self.rect,border_radius=8)
        pygame.draw.rect(surf,brd,self.rect,bw,border_radius=8)
        ls=self.font_lbl.render(self.label,True,TEXT_C if self.selected else DIM_C)
        if self.sublabel:
            ss=self.font_sub.render(self.sublabel,True,SEL_SUB if self.selected else DIM_SUB)
            th=ls.get_height()+3+ss.get_height()
            ly=self.rect.centery-th//2
            surf.blit(ls,ls.get_rect(centerx=self.rect.centerx,top=ly))
            surf.blit(ss,ss.get_rect(centerx=self.rect.centerx,top=ly+ls.get_height()+3))
        else:
            surf.blit(ls,ls.get_rect(center=self.rect.center))
    def update(self,pos): self.hovered=self.rect.collidepoint(pos)
    def hit(self,pos):    return self.rect.collidepoint(pos)


class ActionButton:
    def __init__(self,rect,text,nrm,hvr,font):
        self.rect=pygame.Rect(rect);self.text=text;self.nrm=nrm;self.hvr=hvr
        self.font=font;self.hovered=False
    def draw(self,surf):
        col=self.hvr if self.hovered else self.nrm
        pygame.draw.rect(surf,col,self.rect,border_radius=11)
        pygame.draw.rect(surf,(255,255,255),self.rect,2,border_radius=11)
        lb=self.font.render(self.text,True,(255,255,255))
        surf.blit(lb,lb.get_rect(center=self.rect.center))
    def update(self,pos): self.hovered=self.rect.collidepoint(pos)
    def hit(self,pos):    return self.rect.collidepoint(pos)


class RobotButton:
    """Renkli sol bar + holo rozeti olan robot seçim butonu."""
    def __init__(self, rect, key, lbl, sub, font_lbl, font_sub):
        self.rect      = pygame.Rect(rect)
        self.key       = key;  self.tag = key
        self.label     = lbl;  self.sublabel = sub
        self.font_lbl  = font_lbl;  self.font_sub = font_sub
        self.selected  = False;  self.hovered = False
        _inf           = ROBOT_INFO.get(key, {})
        self.robot_color = _inf.get("color", (60, 100, 200))
        self.is_holo     = _inf.get("holo", False)

    def draw(self, surf):
        rc     = self.robot_color
        is_sel = self.selected
        is_hvr = self.hovered

        # Arka plan — seçiliyse robotun rengiyle hafifçe tonlanmış
        if is_sel:
            bg = (min(rc[0]//4 + 18, 70),
                  min(rc[1]//4 + 28, 80),
                  min(rc[2]//4 + 44, 95))
        elif is_hvr:
            bg = BTN_HVR
        else:
            bg = BTN_NRM
        pygame.draw.rect(surf, bg, self.rect, border_radius=7)

        # Sol renkli şerit
        bar = pygame.Rect(self.rect.x + 2, self.rect.y + 4,
                          4, self.rect.h - 8)
        pygame.draw.rect(surf, rc, bar, border_radius=2)

        # Kenarlık
        if is_sel:
            brd, bw = rc, 2
        elif is_hvr:
            brd, bw = tuple(min(c + 50, 255) for c in rc), 1
        else:
            brd, bw = NRM_BRD, 1
        pygame.draw.rect(surf, brd, self.rect, bw, border_radius=7)

        # Sağ rozet: H = holonomik, N = non-holonomik
        badge_c = (0, 200, 140) if self.is_holo else (200, 130, 40)
        bt      = self.font_sub.render("H" if self.is_holo else "N", True, badge_c)
        surf.blit(bt, (self.rect.right - bt.get_width() - 8,
                       self.rect.centery - bt.get_height() // 2))

        # Metin
        txt_c = TEXT_C  if is_sel else DIM_C
        sub_c = tuple(min(c + 30, 255) for c in rc) if is_sel else DIM_SUB
        ls = self.font_lbl.render(self.label,    True, txt_c)
        ss = self.font_sub.render(self.sublabel, True, sub_c)
        th = ls.get_height() + 2 + ss.get_height()
        ly = self.rect.centery - th // 2
        tx = self.rect.x + 11
        surf.blit(ls, (tx, ly))
        surf.blit(ss, (tx, ly + ls.get_height() + 2))

    def update(self, pos): self.hovered = self.rect.collidepoint(pos)
    def hit(self,   pos): return self.rect.collidepoint(pos)


def _rot(lx,ly,theta,ox,oy):
    c,s=math.cos(theta),math.sin(theta)
    return(int(lx*c-ly*s+ox),int(lx*s+ly*c+oy))


def draw_robot_preview(surf,rtype,cx,cy,R,theta):
    info=ROBOT_INFO.get(rtype,ROBOT_INFO["differential"])
    color=info["color"]; track=info.get("track")
    W=(255,255,255); G=(130,130,145)
    def poly(pts,col,w=0):
        if len(pts)>=3: pygame.draw.polygon(surf,col,pts,w)
    def ln(p1,p2,col,w=2): pygame.draw.line(surf,col,p1,p2,w)

    if rtype in("differential","mecanum"):
        bw=R*1.8;bh=R*1.1;tw=R*1.9;th=R*0.42
        if track:
            for side in(-1,1):
                oy_=side*(bh/2+th/2+1)
                pts=[_rot(-tw/2,oy_-th/2,theta,cx,cy),_rot(tw/2,oy_-th/2,theta,cx,cy),
                     _rot(tw/2,oy_+th/2,theta,cx,cy),_rot(-tw/2,oy_+th/2,theta,cx,cy)]
                poly(pts,track);poly(pts,(track[0]+15,track[1]+15,track[2]+15),1)
                for seg in range(-4,5):
                    sx=seg*tw/9
                    dp=[_rot(sx-1,oy_-th/2+1,theta,cx,cy),_rot(sx+1,oy_-th/2+1,theta,cx,cy),
                        _rot(sx+1,oy_+th/2-1,theta,cx,cy),_rot(sx-1,oy_+th/2-1,theta,cx,cy)]
                    poly(dp,(track[0]-10,track[1]-10,track[2]-10),1)
        if rtype=="mecanum":
            for qx,qy in[(-bw*0.38,-bh*0.38),(bw*0.38,-bh*0.38),(-bw*0.38,bh*0.38),(bw*0.38,bh*0.38)]:
                ln(_rot(qx-R*0.18,qy,theta,cx,cy),_rot(qx+R*0.18,qy,theta,cx,cy),(180,100,255),4)
                ln(_rot(qx-R*0.08,qy-R*0.15,theta,cx,cy),_rot(qx+R*0.08,qy+R*0.15,theta,cx,cy),(140,70,210),2)
        b_pts=[_rot(-bw/2,-bh/2,theta,cx,cy),_rot(bw/2,-bh/2,theta,cx,cy),
               _rot(bw/2,bh/2,theta,cx,cy),_rot(-bw/2,bh/2,theta,cx,cy)]
        poly(b_pts,color);poly(b_pts,W,2)
        tr=int(R*0.5);lc=tuple(min(c+35,255)for c in color)
        pygame.draw.circle(surf,lc,(cx,cy),tr)
        pygame.draw.circle(surf,W,(cx,cy),tr,2)
        ln((cx,cy),_rot(R*1.5,0,theta,cx,cy),W,4)
        pygame.draw.circle(surf,G,_rot(R*1.5,0,theta,cx,cy),5)
        for ey in(-bh*0.2,bh*0.2):
            pygame.draw.circle(surf,(0,220,120),_rot(bw*0.28,ey,theta,cx,cy),4)

    elif rtype=="omni":
        pygame.draw.circle(surf,color,(cx,cy),R)
        pygame.draw.circle(surf,W,(cx,cy),R,2)
        pygame.draw.circle(surf,tuple(max(c-30,0)for c in color),(cx,cy),int(R*0.5))
        for i in range(3):
            ang=theta+i*(2*math.pi/3)
            inn=(int(cx+R*0.5*math.cos(ang)),int(cy+R*0.5*math.sin(ang)))
            out=(int(cx+R*1.1*math.cos(ang)),int(cy+R*1.1*math.sin(ang)))
            ln(inn,out,W,3)
            perp=ang+math.pi/2;ww=R*0.22;wh=R*0.35
            w_pts=[(int(out[0]+ww*math.cos(ang)-wh*math.cos(perp)),int(out[1]+ww*math.sin(ang)-wh*math.sin(perp))),
                   (int(out[0]-ww*math.cos(ang)-wh*math.cos(perp)),int(out[1]-ww*math.sin(ang)-wh*math.sin(perp))),
                   (int(out[0]-ww*math.cos(ang)+wh*math.cos(perp)),int(out[1]-ww*math.sin(ang)+wh*math.sin(perp))),
                   (int(out[0]+ww*math.cos(ang)+wh*math.cos(perp)),int(out[1]+ww*math.sin(ang)+wh*math.sin(perp)))]
            poly(w_pts,(0,200,220));poly(w_pts,W,1)
            for j in range(-2,3):
                f=j/4
                p1=(int(w_pts[0][0]*(0.5-f/2)+w_pts[1][0]*(0.5+f/2)),int(w_pts[0][1]*(0.5-f/2)+w_pts[1][1]*(0.5+f/2)))
                p2=(int(w_pts[3][0]*(0.5-f/2)+w_pts[2][0]*(0.5+f/2)),int(w_pts[3][1]*(0.5-f/2)+w_pts[2][1]*(0.5+f/2)))
                ln(p1,p2,(0,240,255),1)
        pygame.draw.circle(surf,W,(cx,cy),6)
        pygame.draw.circle(surf,(255,220,0),_rot(int(R*0.85),0,theta,cx,cy),7)

    elif rtype=="ackermann":
        bw=R*1.6;bh=R*1.0;cw=R*1.0;ch=R*0.55;tw_=R*0.22;th_=R*0.42
        wheel_defs=[(-bw*0.38,-bh*0.55,0.0),(bw*0.38,-bh*0.55,0.25),
                    (-bw*0.38,bh*0.55,0.0),(bw*0.38,bh*0.55,0.25)]
        for wx,wy,steer in wheel_defs:
            sc,ss=math.cos(steer),math.sin(steer)
            fwd=(sc,ss);perp=(-ss,sc)
            lp=[(wx+tw_*fwd[0]+th_*perp[0],wy+tw_*fwd[1]+th_*perp[1]),
                (wx+tw_*fwd[0]-th_*perp[0],wy+tw_*fwd[1]-th_*perp[1]),
                (wx-tw_*fwd[0]-th_*perp[0],wy-tw_*fwd[1]-th_*perp[1]),
                (wx-tw_*fwd[0]+th_*perp[0],wy-tw_*fwd[1]+th_*perp[1])]
            w_pts=[_rot(lx,ly,theta,cx,cy)for lx,ly in lp]
            poly(w_pts,(30,30,35));poly(w_pts,G,1)
        b_pts=[_rot(-bw/2,-bh/2,theta,cx,cy),_rot(bw/2,-bh/2,theta,cx,cy),
               _rot(bw/2,bh/2,theta,cx,cy),_rot(-bw/2,bh/2,theta,cx,cy)]
        poly(b_pts,color);poly(b_pts,W,2)
        cb=[_rot(-cw/2,-ch/2,theta,cx,cy),_rot(cw/2,-ch/2,theta,cx,cy),
            _rot(cw/2,ch/2,theta,cx,cy),_rot(-cw/2,ch/2,theta,cx,cy)]
        poly(cb,tuple(min(c+25,255)for c in color));poly(cb,W,1)
        ln(_rot(bw*0.35,-bh*0.55-4,theta,cx,cy),_rot(bw*0.35,bh*0.55+4,theta,cx,cy),G,2)
        ln(_rot(-bw*0.35,-bh*0.55-4,theta,cx,cy),_rot(-bw*0.35,bh*0.55+4,theta,cx,cy),G,2)
        fp=_rot(R*1.6,0,theta,cx,cy);ln((cx,cy),fp,(255,220,0),2)
        pygame.draw.circle(surf,(255,220,0),fp,5)

    elif rtype=="drone":
        al=R*1.0;rr=int(R*0.32);br=int(R*0.38)
        for i in range(4):
            aa=theta+math.pi/4+i*math.pi/2
            tx=int(cx+al*math.cos(aa));ty=int(cy+al*math.sin(aa))
            bx_=int(cx+br*math.cos(aa));by_=int(cy+br*math.sin(aa))
            pygame.draw.line(surf,G,(bx_,by_),(tx,ty),3)
            pygame.draw.circle(surf,(40,40,50),(tx,ty),rr)
            pygame.draw.circle(surf,color,(tx,ty),rr,2)
            ba=theta*4+i*math.pi/2
            for b2 in[ba,ba+math.pi/2]:
                bx1=int(tx+rr*0.85*math.cos(b2));by1=int(ty+rr*0.85*math.sin(b2))
                bx2=int(tx-rr*0.85*math.cos(b2));by2=int(ty-rr*0.85*math.sin(b2))
                pygame.draw.line(surf,color,(bx1,by1),(bx2,by2),3)
        pygame.draw.circle(surf,color,(cx,cy),br)
        pygame.draw.circle(surf,W,(cx,cy),br,2)
        pygame.draw.circle(surf,(20,20,25),(cx,cy),int(br*0.5))
        pygame.draw.circle(surf,(0,200,255),(cx,cy),int(br*0.5),1)
        led=_rot(br,0,theta,cx,cy)
        pygame.draw.circle(surf,(255,80,80),led,5)

    elif rtype=="vtol":
        fl=R*1.6;fh=R*0.3;wl=R*1.4;tl=R*0.5;th2=R*0.18
        wp=[_rot(-fl*0.1,-wl,theta,cx,cy),_rot(fl*0.2,-wl,theta,cx,cy),
            _rot(fl*0.2,wl,theta,cx,cy),_rot(-fl*0.1,wl,theta,cx,cy)]
        poly(wp,tuple(max(c-20,0)for c in color));poly(wp,W,1)
        fp2=[_rot(-fl/2,-fh/2,theta,cx,cy),_rot(fl/2,-fh/2,theta,cx,cy),
             _rot(fl/2,fh/2,theta,cx,cy),_rot(-fl/2,fh/2,theta,cx,cy)]
        poly(fp2,color);poly(fp2,W,2)
        tp=[_rot(-fl/2,-tl,theta,cx,cy),_rot(-fl/2+th2,-tl,theta,cx,cy),
            _rot(-fl/2+th2,tl,theta,cx,cy),_rot(-fl/2,tl,theta,cx,cy)]
        poly(tp,tuple(min(c+20,255)for c in color));poly(tp,W,1)
        rotor_r=int(R*0.22)
        for side in(-1,1):
            rx=_rot(0,side*wl*0.82,theta,cx,cy)
            pygame.draw.circle(surf,(30,30,40),rx,rotor_r)
            pygame.draw.circle(surf,(0,255,180),rx,rotor_r,2)
            ba2=theta*3+side*0.5
            for b2 in[ba2,ba2+math.pi/2]:
                bx1=int(rx[0]+rotor_r*0.8*math.cos(b2));by1=int(rx[1]+rotor_r*0.8*math.sin(b2))
                bx2=int(rx[0]-rotor_r*0.8*math.cos(b2));by2=int(rx[1]-rotor_r*0.8*math.sin(b2))
                pygame.draw.line(surf,(0,220,150),(bx1,by1),(bx2,by2),2)
        nose=_rot(fl*0.55,0,theta,cx,cy)
        pygame.draw.circle(surf,(50,50,60),nose,int(R*0.18))
        pygame.draw.circle(surf,(255,220,0),nose,int(R*0.18),2)

    elif rtype=="fixedwing":
        fl=R*1.8;fh=R*0.28;wl=R*1.6;sl=R*0.55
        for side in(-1,1):
            wpts=[_rot(-fl*0.05,side*fh*0.5,theta,cx,cy),_rot(fl*0.20,side*fh*0.5,theta,cx,cy),
                  _rot(-fl*0.18,side*(fh*0.5+wl),theta,cx,cy),_rot(-fl*0.30,side*(fh*0.5+wl),theta,cx,cy)]
            poly(wpts,color);poly(wpts,W,1)
            el=[_rot(-fl*0.18,side*(fh*0.5+wl*0.65),theta,cx,cy),_rot(-fl*0.10,side*(fh*0.5+wl*0.65),theta,cx,cy),
                _rot(-fl*0.10,side*(fh*0.5+wl*0.98),theta,cx,cy),_rot(-fl*0.22,side*(fh*0.5+wl*0.98),theta,cx,cy)]
            poly(el,tuple(min(c+40,255)for c in color));poly(el,W,1)
        fpts=[_rot(-fl/2,-fh/2,theta,cx,cy),_rot(fl/2,-fh/2,theta,cx,cy),
              _rot(fl/2,fh/2,theta,cx,cy),_rot(-fl/2,fh/2,theta,cx,cy)]
        poly(fpts,tuple(min(c+20,255)for c in color));poly(fpts,W,2)
        for side in(-1,1):
            tp=[_rot(-fl*0.38,side*fh*0.4,theta,cx,cy),_rot(-fl*0.30,side*fh*0.4,theta,cx,cy),
                _rot(-fl*0.38,side*(fh*0.4+sl),theta,cx,cy)]
            poly(tp,tuple(max(c-20,0)for c in color));poly(tp,W,1)
        vt=[_rot(-fl*0.45,0,theta,cx,cy),_rot(-fl*0.25,0,theta,cx,cy),_rot(-fl*0.42,-sl*0.7,theta,cx,cy)]
        poly(vt,tuple(min(c+30,255)for c in color));poly(vt,W,1)
        nose=_rot(fl*0.52,0,theta,cx,cy)
        pygame.draw.circle(surf,(40,40,50),nose,int(R*0.15))
        pygame.draw.circle(surf,(255,220,0),nose,int(R*0.15),2)
        pa=theta*5
        for p2 in[pa,pa+math.pi/2]:
            px1=int(nose[0]+R*0.24*math.cos(p2));py1=int(nose[1]+R*0.24*math.sin(p2))
            px2=int(nose[0]-R*0.24*math.cos(p2));py2=int(nose[1]-R*0.24*math.sin(p2))
            pygame.draw.line(surf,(200,200,220),(px1,py1),(px2,py2),2)

    elif rtype=="quadruped":
        bw=R*1.8; bh_=R*0.85; lh=R*0.52
        b_pts=[_rot(-bw/2,-bh_/2,theta,cx,cy),_rot(bw/2,-bh_/2,theta,cx,cy),
               _rot(bw/2,bh_/2,theta,cx,cy),_rot(-bw/2,bh_/2,theta,cx,cy)]
        poly(b_pts,color); poly(b_pts,W,2)
        # 4 bacak: ön/arka x sol/sağ
        for lx in(-bw*0.30,bw*0.30):
            for side in(-1,1):
                hip =_rot(lx,side*bh_/2,            theta,cx,cy)
                knee=_rot(lx,side*(bh_/2+lh*0.52),  theta,cx,cy)
                foot=_rot(lx,side*(bh_/2+lh),       theta,cx,cy)
                ln(hip,knee,G,3); ln(knee,foot,W,3)
                pygame.draw.circle(surf,(200,200,210),foot,4)
        # Baş
        hx=bw*0.62; hr=int(R*0.38)
        hc=_rot(hx,0,theta,cx,cy)
        pygame.draw.circle(surf,tuple(min(c+25,255)for c in color),hc,hr)
        pygame.draw.circle(surf,W,hc,hr,2)
        # Kulaklar
        for ey in(-hr*0.72,hr*0.72):
            ec=_rot(hx+hr*0.28,ey,theta,cx,cy)
            pygame.draw.circle(surf,tuple(max(c-15,0)for c in color),ec,int(hr*0.28))
            pygame.draw.circle(surf,W,ec,int(hr*0.28),1)
        # Gözler
        pygame.draw.circle(surf,(0,230,110),_rot(hx+hr*0.15,-hr*0.32,theta,cx,cy),3)
        pygame.draw.circle(surf,(0,230,110),_rot(hx+hr*0.15, hr*0.32,theta,cx,cy),3)
        # Kuyruk
        ln(_rot(-bw/2,0,theta,cx,cy),_rot(-bw/2-R*0.45,-R*0.38,theta,cx,cy),G,2)
        pygame.draw.circle(surf,(200,200,200),_rot(-bw/2-R*0.45,-R*0.38,theta,cx,cy),3)

    elif rtype=="hexapod":
        pygame.draw.circle(surf,color,(cx,cy),R)
        pygame.draw.circle(surf,W,    (cx,cy),R,2)
        # 6 bacak 60° aralıklı, diz bükümlü
        for i in range(6):
            base_a=theta+i*(math.pi/3)
            hip=(int(cx+R*0.82*math.cos(base_a)),int(cy+R*0.82*math.sin(base_a)))
            knee_a=base_a+(0.45 if i%2==0 else -0.45)
            knee=(int(cx+R*1.55*math.cos(knee_a)),int(cy+R*1.55*math.sin(knee_a)))
            foot=(int(cx+R*1.90*math.cos(base_a)),int(cy+R*1.90*math.sin(base_a)))
            ln(hip,knee,G,3); ln(knee,foot,color,2)
            pygame.draw.circle(surf,(200,200,210),foot,4)
        pygame.draw.circle(surf,tuple(min(c+15,255)for c in color),(cx,cy),int(R*0.52))
        pygame.draw.circle(surf,(15,20,25),(cx,cy),int(R*0.28))
        # Yön göstergesi
        pygame.draw.circle(surf,(255,200,50),_rot(int(R*0.90),0,theta,cx,cy),7)
        ln((cx,cy),_rot(int(R*0.74),0,theta,cx,cy),(255,200,50),2)

    elif rtype=="snake":
        n_segs=6; sr_=int(R*0.34)
        # Kuyruktan başa çiz (baş en üstte)
        for i in range(n_segs-1,-1,-1):
            t_=i/(n_segs-1)
            seg_x=R*(0.82-t_*1.72)
            seg_y=math.sin(i*1.1+theta*4)*R*0.30*(1-t_*0.20)
            sc_=tuple(max(int(c*(1-t_*0.32)),0)for c in color)
            sr_i=max(int(sr_*(1-t_*0.08)),3)
            sc=_rot(seg_x,seg_y,theta,cx,cy)
            pygame.draw.circle(surf,sc_,sc,sr_i)
            pygame.draw.circle(surf,tuple(min(c+18,255)for c in sc_),sc,sr_i,1)
        # Baş vurgusu
        head_c=_rot(int(R*0.82),0,theta,cx,cy)
        pygame.draw.circle(surf,tuple(min(c+40,255)for c in color),head_c,int(sr_*1.12))
        pygame.draw.circle(surf,W,head_c,int(sr_*1.12),2)
        # Gözler
        pygame.draw.circle(surf,(0,235,120),_rot(int(R*0.88),-int(sr_*0.42),theta,cx,cy),3)
        pygame.draw.circle(surf,(0,235,120),_rot(int(R*0.88), int(sr_*0.42),theta,cx,cy),3)

    elif rtype=="bipedal":
        bw_=R*0.70; bh_=R*0.82
        # Gövde (torso)
        t_pts=[_rot(-bw_/2,-bh_/2,theta,cx,cy),_rot(bw_/2,-bh_/2,theta,cx,cy),
               _rot(bw_/2, bh_/2,theta,cx,cy),_rot(-bw_/2, bh_/2,theta,cx,cy)]
        poly(t_pts,color); poly(t_pts,W,2)
        # Kafa
        hr=int(R*0.30)
        hc=_rot(0,-(bh_/2+R*0.38),theta,cx,cy)
        pygame.draw.circle(surf,tuple(min(c+28,255)for c in color),hc,hr)
        pygame.draw.circle(surf,W,hc,hr,2)
        # Gözler
        pygame.draw.circle(surf,(0,200,255),_rot(-R*0.10,-(bh_/2+R*0.38)-R*0.08,theta,cx,cy),3)
        pygame.draw.circle(surf,(0,200,255),_rot( R*0.10,-(bh_/2+R*0.38)-R*0.08,theta,cx,cy),3)
        # Kollar
        for side in(-1,1):
            ln(_rot(side*bw_*0.48,-bh_*0.32,theta,cx,cy),
               _rot(side*bw_*0.82, bh_*0.04,theta,cx,cy),W,2)
        # Bacaklar
        for side in(-1,1):
            hip =_rot(side*bw_*0.32, bh_*0.44,theta,cx,cy)
            knee=_rot(side*bw_*0.30, bh_*0.88,theta,cx,cy)
            foot=_rot(side*bw_*0.22, bh_*1.18,theta,cx,cy)
            ln(hip,knee,W,3); ln(knee,foot,W,3)
            pygame.draw.circle(surf,(200,200,210),foot,5)
        # Yön göstergesi
        pygame.draw.circle(surf,(255,200,50),_rot(bw_*0.65,-bh_*0.22,theta,cx,cy),5)


def draw_preview_panel(surf,rtype,panel_rect,tick,fonts):
    info=ROBOT_INFO.get(rtype,ROBOT_INFO["differential"]); r=panel_rect
    pygame.draw.rect(surf,PREV_BG,r,border_radius=12)
    pygame.draw.rect(surf,SEL_BRD,r,2,border_radius=12)
    dh=int(r.height*0.52);cx_=r.centerx;cy_=r.top+dh//2
    rob_r=int(min(r.width,dh)*0.28);theta=tick*0.018
    grid=pygame.Surface((r.width-4,dh),pygame.SRCALPHA)
    for gx in range(0,r.width,32): pygame.draw.line(grid,(255,255,255,12),(gx,0),(gx,dh))
    for gy in range(0,dh,32): pygame.draw.line(grid,(255,255,255,12),(0,gy),(r.width-4,gy))
    surf.blit(grid,(r.left+2,r.top+2))
    pygame.draw.circle(surf,(15,25,45),(cx_+6,cy_+8),rob_r+8)
    draw_robot_preview(surf,rtype,cx_,cy_,rob_r,theta)
    sy=r.top+dh+10
    pygame.draw.line(surf,SEP_C,(r.left+16,sy),(r.right-16,sy),1)
    ts=fonts["title"].render(info["title"],True,TITLE_C)
    surf.blit(ts,ts.get_rect(centerx=r.centerx,top=sy+12))
    ty=sy+12+ts.get_height()+12
    for prop in info["props"]:
        bs=fonts["prop"].render(f"  >>  {prop}",True,TEXT_C)
        surf.blit(bs,(r.left+20,ty));ty+=bs.get_height()+6
    holo="Holonomik" if info.get("holo") else "Non-Holonomik"
    hc=(0,220,140) if info.get("holo") else(255,165,40)
    ls=fonts["label"].render(holo,True,hc)
    lr=ls.get_rect(centerx=r.centerx,bottom=r.bottom-12)
    surf.blit(ls,lr);pygame.draw.rect(surf,hc,lr.inflate(16,6),1,border_radius=5)


def _draw_grid(surf,W,H,tick):
    step=55;t=tick*0.3
    for gx in range(0,W+step,step):
        for gy in range(0,H+step,step):
            ox=int((gx+t)%W);oy=int((gy+t*0.5)%H)
            surf.set_at((ox,oy),GRID_C)


def _draw_section(surf,mx,sy,pw,sh,title,font):
    r=pygame.Rect(mx,sy,pw,sh)
    pygame.draw.rect(surf,PANEL_BG,r,border_radius=10)
    pygame.draw.rect(surf,PANEL_BRD,r,1,border_radius=10)
    lbl=font.render(f"  {title}",True,SECT_C);lh=lbl.get_height()+6
    head=pygame.Rect(mx,sy,pw,lh+4)
    pygame.draw.rect(surf,(22,38,68),head,border_top_left_radius=10,border_top_right_radius=10)
    pygame.draw.line(surf,SEP_C,(mx,sy+lh+4),(mx+pw,sy+lh+4),1)
    surf.blit(lbl,(mx+8,sy+4))


def _draw_pad_ctrl(surf,minus_r,plus_r,value,pad_y,ctrl_h,ctrl_w,gap,mx,font_lg,font_md,mp):
    for rect,lbl in[(minus_r,"-"),(plus_r,"+")]:
        col=BTN_HVR if rect.collidepoint(mp) else BTN_NRM
        pygame.draw.rect(surf,col,rect,border_radius=7)
        pygame.draw.rect(surf,NRM_BRD,rect,1,border_radius=7)
        s=font_lg.render(lbl,True,TEXT_C);surf.blit(s,s.get_rect(center=rect.center))
    val_x=mx+ctrl_w*2+gap*2+6
    vs=font_lg.render(str(value),True,TITLE_C)
    surf.blit(vs,vs.get_rect(midleft=(val_x,pad_y+ctrl_h//2)))
    dx=val_x+vs.get_width()+16
    ds=font_md.render(f"{value} hucre  ({value*0.5:.1f} m)  --  engel etrafinda guvenlik boslugu",True,DIM_C)
    surf.blit(ds,ds.get_rect(midleft=(dx,pad_y+ctrl_h//2)))


_SENSOR_CFG = [
    ("lidar_range", "Menzil",  0.5,  2.0, 15.0, "{:.1f}m"),
    ("lidar_beams", "Isin",   30,   60,  360,   "{}"),
    ("lidar_noise", "Gurultu", 0.01, 0.0, 0.20,  "{:.2f}"),
]


def _cfg(sel, padding, mode, s_params=None):
    d = {**sel, "padding": padding, "mode": mode}
    if s_params:
        d.update(s_params)
    return d


def run_menu(screen=None):
    if screen is None:
        pygame.init()
        info=pygame.display.Info();W,H=info.current_w,info.current_h
        screen=pygame.display.set_mode((W,H),pygame.NOFRAME)
    else:
        W,H=screen.get_size()
    pygame.display.set_caption("Otonom Navigasyon -- Ana Menu")
    base=max(10,H//72)
    fn_sm=pygame.font.SysFont("consolas",base-2)
    fn_md=pygame.font.SysFont("consolas",base,bold=True)
    fn_sect=pygame.font.SysFont("consolas",base+1,bold=True)
    fn_lg=pygame.font.SysFont("consolas",base+4,bold=True)
    fn_title=pygame.font.SysFont("consolas",base+7,bold=True)
    fn_pt=pygame.font.SysFont("consolas",base+3,bold=True)
    pfonts={"title":fn_pt,"prop":fn_sm,"label":fn_md}
    clk=pygame.time.Clock()
    sel={"robot":"differential","planner":"astar","metric":"euclidean","criteria":"shortest"}
    padding=1
    mx    = int(W*0.022)
    gap   = int(H*0.010)
    bh    = int(H*0.052)                  # standart buton yüksekliği
    bh_r  = int(H*0.040)                  # robot buton y. (kompakt — 3 satır)
    bh_m  = int(H*0.038)                  # metrik buton y. (2-satır ızgara)
    _lbl_h= max(12, int(H*0.016))         # kategori etiket yüksekliği
    _hdr  = int(H*0.030)                  # bölüm başlık ofseti
    th_   = int(H*0.075)                  # başlık alanı

    prev_w=int(W*0.23); prev_x=W-mx-prev_w; prev_y=int(H*0.09)
    prev_rect=pygame.Rect(prev_x,prev_y,prev_w,H-prev_y-int(H*0.10))
    lw=prev_x-2*mx

    # Dişey konumlar
    _row_step = bh_r + _lbl_h + gap
    ph_robot  = _hdr + len(ROBOT_ROWS)*_row_step
    y_robot   = th_ + gap

    ph_plan   = _hdr + bh*2 + int(H*0.040) + gap + 14
    y_plan    = y_robot + ph_robot + gap

    tile_h    = _hdr + bh_m*2 + gap + 4
    y_metric  = y_plan + ph_plan + gap

    ph_c      = _hdr + bh + gap
    y_crit    = y_metric + tile_h + gap

    ctrl_h    = int(bh*0.78); ctrl_w = int(bh*0.78)
    _sensor_row_h = ctrl_h + 8
    ph_pad    = _hdr + ctrl_h + gap + _sensor_row_h + 12
    y_pad     = y_crit + ph_c + gap

    y_action  = H - max(68, int(H*0.085))

    fn_cat = pygame.font.SysFont("consolas", max(9, base-2), bold=True)

    # Robot butonları — 3 kategori satırı
    robot_btns = []
    _cat_infos = []   # (metin, y, renk)
    CAT_COLORS = [(60,130,255),(50,190,110),(220,130,50)]
    for row_i,(cat_name,row_opts) in enumerate(ROBOT_ROWS):
        lbl_y = y_robot + _hdr + row_i*_row_step
        row_y = lbl_y + _lbl_h
        _cat_infos.append((cat_name, lbl_y, CAT_COLORS[row_i % len(CAT_COLORS)]))
        n     = len(row_opts)
        bw_r  = (lw - gap*(n-1)) // n
        for col_i,(key,lbl,sub) in enumerate(row_opts):
            bx = mx + col_i*(bw_r+gap)
            b  = RobotButton((bx,row_y,bw_r,bh_r), key, lbl, sub,
                             font_lbl=fn_md, font_sub=fn_sm)
            b.selected = (key == sel["robot"])
            robot_btns.append(b)

    # Planlayıcı butonları — global + local
    _row1_y = y_plan + _hdr
    _row2_y = _row1_y + bh + gap + 8

    def make_row(opts,y,sk):
        n=len(opts); bw=(lw-gap*(n-1))//n; btns=[]
        for i,(key,lbl,sub) in enumerate(opts):
            bx=mx+i*(bw+gap)
            b=Button((bx,y,bw,bh),lbl,sub,tag=key,font_lbl=fn_md,font_sub=fn_sm)
            b.selected=(key==sel[sk]); btns.append(b)
        return btns

    global_planner_btns = make_row(GLOBAL_PLANNER_OPTS, _row1_y, "planner")
    local_planner_btns  = make_row(LOCAL_PLANNER_OPTS,  _row2_y, "planner")
    planner_btns        = global_planner_btns + local_planner_btns

    criteria_btns = make_row(CRITERIA_OPTS, y_crit+_hdr, "criteria")

    metric_btns=[]
    n_cols=4; bw_m_=(lw-gap*(n_cols-1))//n_cols
    for i,(key,lbl,sub) in enumerate(METRIC_OPTS):
        row,col=divmod(i,n_cols)
        bx=mx+col*(bw_m_+gap); by=y_metric+_hdr+row*(bh_m+gap)
        b=Button((bx,by,bw_m_,bh_m),lbl,sub,tag=key,font_lbl=fn_md,font_sub=fn_sm)
        b.selected=(key==sel["metric"]); metric_btns.append(b)

    pad_y_    = y_pad + _hdr
    pad_minus = pygame.Rect(mx,            pad_y_, ctrl_w, ctrl_h)
    pad_plus  = pygame.Rect(mx+ctrl_w+gap, pad_y_, ctrl_w, ctrl_h)

    # Sensör parametreleri
    s_params = {"lidar_range": 8.0, "lidar_beams": 180, "lidar_noise": 0.04}
    sensor_ctrl_y = pad_y_ + ctrl_h + gap
    _sens_gw = (lw - gap * 2) // 3
    sensor_minus = []
    sensor_plus  = []
    for _si in range(3):
        _gx = mx + _si * (_sens_gw + gap)
        sensor_minus.append(pygame.Rect(_gx,               sensor_ctrl_y, ctrl_w, ctrl_h))
        sensor_plus.append( pygame.Rect(_gx + ctrl_w + gap, sensor_ctrl_y, ctrl_w, ctrl_h))

    act_w=int((lw - 2*gap) // 3); act_h=int(H*0.062)
    btn_start =ActionButton((mx,                    y_action,act_w,act_h),"BASLA  [ENTER]",GREEN_N, GREEN_H, fn_lg)
    btn_assgn =ActionButton((mx+act_w+gap,           y_action,act_w,act_h),"ODEV MODU",    ORANGE_N,ORANGE_H,fn_lg)
    btn_report=ActionButton((mx+2*(act_w+gap),        y_action,act_w,act_h),"RAPOR MODU",   BLUE_N,  BLUE_H,  fn_lg)

    all_groups={"robot":robot_btns,"planner":planner_btns,
                "metric":metric_btns,"criteria":criteria_btns}
    sections=[
        (y_robot,  ph_robot, "ROBOT TIPI"),
        (y_plan,   ph_plan,  "YOL PLANLAYICI"),
        (y_metric, tile_h,   "MESAFE METRIGI"),
        (y_crit,   ph_c,     "YOL KRITERI"),
        (y_pad,    ph_pad,   "ENGEL DOLGUSU / SENSOR"),
    ]

    tick=0
    while True:
        tick+=1; mp=pygame.mouse.get_pos()
        for event in pygame.event.get():
            if event.type==pygame.QUIT: return None
            if event.type==pygame.KEYDOWN:
                if event.key in(pygame.K_ESCAPE,pygame.K_F4): return None
                if event.key==pygame.K_RETURN: return _cfg(sel,padding,"sim",s_params)
                if event.key==pygame.K_F12:
                    _ss_dir = os.path.join("outputs", "screenshots")
                    os.makedirs(_ss_dir, exist_ok=True)
                    _ss_ts  = datetime.now().strftime("%Y%m%d_%H%M%S")
                    _ss_path= os.path.join(_ss_dir, f"menu_{_ss_ts}.png")
                    pygame.image.save(screen, _ss_path)
                    print(f"  Menu ekran goruntusu: {_ss_path}")
            if event.type==pygame.MOUSEBUTTONDOWN and event.button==1:
                _is_local=sel["planner"] in LOCAL_PLANNER_KEYS
                for gk,btns in all_groups.items():
                    for b in btns:
                        if b.hit(mp):
                            if _is_local and gk in ("metric","criteria"):
                                break
                            for bb in btns: bb.selected=False
                            b.selected=True; sel[gk]=b.tag
                if pad_minus.collidepoint(mp): padding=max(0,padding-1)
                if pad_plus.collidepoint(mp):  padding=min(5,padding+1)
                for _si,(_sp,_sl,_ss,_smn,_smx,_sfmt) in enumerate(_SENSOR_CFG):
                    if sensor_minus[_si].collidepoint(mp):
                        s_params[_sp]=max(_smn, round(s_params[_sp]-_ss, 4))
                    if sensor_plus[_si].collidepoint(mp):
                        s_params[_sp]=min(_smx, round(s_params[_sp]+_ss, 4))
                if btn_start.hit(mp): return _cfg(sel,padding,"sim",s_params)
                if btn_assgn.hit(mp): return _cfg(sel,padding,"assignment",s_params)
                if btn_report.hit(mp): return _cfg(sel,padding,"report",s_params)

        screen.fill(BG); _draw_grid(screen,W,H,tick)

        # Başlık
        t1=fn_title.render("OTONOM NAVIGASYON SIMULATORU",True,TITLE_C)
        t2=fn_sect.render(
            "LiDAR | EKF | 11 Robot: Tekerlek + Bacakli + Hava  |  5+5 Planlayi",
            True,DIM_C)
        cxl=mx+lw//2
        screen.blit(t1,t1.get_rect(centerx=cxl,top=int(H*0.016)))
        screen.blit(t2,t2.get_rect(centerx=cxl,top=int(H*0.016)+t1.get_height()+3))

        is_local=sel["planner"] in LOCAL_PLANNER_KEYS

        # Bölüm arka planları
        for sy,sh,st in sections: _draw_section(screen,mx,sy,lw,sh,st,fn_sect)

        # Robot kategori etiketleri — renkli pill tasarım
        for cat_name,lbl_y,cat_col in _cat_infos:
            sep_y = lbl_y + _lbl_h // 2
            pygame.draw.line(screen, tuple(c//4 for c in cat_col),
                             (mx+4, sep_y), (mx+lw-4, sep_y), 1)
            cs    = fn_cat.render(f" {cat_name} ", True, cat_col)
            pill  = pygame.Rect(mx+8, lbl_y, cs.get_width()+8, _lbl_h)
            pygame.draw.rect(screen, tuple(c//5 for c in cat_col), pill, border_radius=4)
            pygame.draw.rect(screen, tuple(c//2 for c in cat_col), pill, 1, border_radius=4)
            screen.blit(cs, cs.get_rect(center=pill.center))

        # Planlayıcı global/local mini etiketleri ve ayırıcı
        _glbl_txt=fn_sm.render("GLOBAL",True,DIM_C)
        _lcl_txt =fn_sm.render("LOCAL (LiDAR Reaktif)",True,(80,160,255))
        screen.blit(_glbl_txt,_glbl_txt.get_rect(left=mx+10,bottom=global_planner_btns[0].rect.top-1))
        screen.blit(_lcl_txt, _lcl_txt.get_rect(left=mx+10, bottom=local_planner_btns[0].rect.top-1))
        _sep_y=global_planner_btns[0].rect.bottom+gap//2
        pygame.draw.line(screen,SEP_C,(mx+8,_sep_y),(mx+lw-8,_sep_y),1)

        # Tüm butonlar
        for gk,btns in all_groups.items():
            for b in btns: b.update(mp); b.draw(screen)

        # Local modda metrik/kriter yarı-saydam kaplama
        if is_local:
            for _sy,_sh in[(y_metric,tile_h),(y_crit,ph_c)]:
                _ov=pygame.Surface((lw,_sh),pygame.SRCALPHA)
                _ov.fill((10,16,28,165))
                screen.blit(_ov,(mx,_sy))
                _na=fn_md.render("Local modda kullanilmaz",True,(65,90,140))
                screen.blit(_na,_na.get_rect(centerx=mx+lw//2,centery=_sy+_sh//2))

        _draw_pad_ctrl(screen,pad_minus,pad_plus,padding,pad_y_,ctrl_h,ctrl_w,gap,mx,fn_lg,fn_md,mp)

        # Sensör parametresi kontrolleri
        for _si,(_sp,_sl,_ss,_smn,_smx,_sfmt) in enumerate(_SENSOR_CFG):
            mr_  = sensor_minus[_si]
            pr_  = sensor_plus[_si]
            _gx  = mx + _si * (_sens_gw + gap)
            for _r,_l in [(mr_,"-"),(pr_,"+")]:
                _bc=BTN_HVR if _r.collidepoint(mp) else BTN_NRM
                pygame.draw.rect(screen,_bc,_r,border_radius=5)
                pygame.draw.rect(screen,NRM_BRD,_r,1,border_radius=5)
                _ts=fn_lg.render(_l,True,TEXT_C)
                screen.blit(_ts,_ts.get_rect(center=_r.center))
            _val_x = _gx + ctrl_w*2 + gap*2 + 4
            _vs = fn_lg.render(_sfmt.format(s_params[_sp]), True, TITLE_C)
            screen.blit(_vs,_vs.get_rect(midleft=(_val_x, sensor_ctrl_y+ctrl_h//2)))
            _lbl_s = fn_sm.render(_sl+":", True, DIM_C)
            screen.blit(_lbl_s,_lbl_s.get_rect(
                midleft=(_val_x+_vs.get_width()+12, sensor_ctrl_y+ctrl_h//2)))
        btn_start.update(mp); btn_start.draw(screen)
        btn_assgn.update(mp); btn_assgn.draw(screen)
        btn_report.update(mp); btn_report.draw(screen)
        draw_preview_panel(screen,sel["robot"],prev_rect,tick,pfonts)

        if is_local:
            st_txt=(f"  Robot:{sel['robot']}  Plan:{sel['planner']} [LOCAL/LiDAR]  "
                    f"Metrik:N/A  Kriter:N/A  "
                    f"Dolgu:{padding}x({padding*0.5:.1f}m)  |  ESC:Cikis")
        else:
            st_txt=(f"  Robot:{sel['robot']}  Plan:{sel['planner']}  "
                    f"Metrik:{sel['metric']}  Kriter:{sel['criteria']}  "
                    f"Dolgu:{padding}x({padding*0.5:.1f}m)  |  ESC:Cikis")
        ss=fn_sm.render(st_txt,True,DIM_C)
        screen.blit(ss,ss.get_rect(left=mx,bottom=H-5))
        pygame.display.flip(); clk.tick(60)
