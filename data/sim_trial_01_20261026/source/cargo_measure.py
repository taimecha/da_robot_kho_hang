#!/usr/bin/env python3
"""Do RONG x CAO hang truoc mat xe: LiDAR cho mat phang mat truoc hang, camera
nhin thang cho cac mep.

    python3 cargo_measure.py agv1 [--save anh.png] [--sim]   # do mot lan (xe dung yen)
    (--sim: dung camera mac dinh trong cargo.yaml, bo qua camera_agvN.yaml)

Cach tinh (he base_footprint: x truoc, y trai, z len):
 1. LiDAR (cargo.yaml lidar.*): diem trong +-half_fov_deg phia truoc, gan hon
    max_range, gop nhieu vong quet -> cum gan truc xe nhat -> RANSAC duong
    thang = mat truoc hang: n.p = d (mat phang dung), goc lech, hai mut.
 2. Anh: khu meo (camera.dist) -> tach hang khoi nen theo DO BAO HOA (bia carton
    co mau, tuong/san xam) -> thanh phan chua diem giua mat truoc (chieu tu LiDAR).
 3. Mep trai/phai (trung vi cac hang giua), mep tren/duoi (trung vi cac cot giua).
    Neu mep lech > EDGE_TOL so voi mut LiDAR chieu len anh (hop xoay -> thay
    them mat ben) thi lay vach doi do sang manh nhat gan mut LiDAR.
 4. Tia tu tam camera qua tung mep cat mat phang mat truoc -> diem 3D -> rong
    (ngang), cao (doc). Day mat truoc phai o z ~ 0 (dung tren san).
"""
import math
import os
import sys
import time

import cv2
import numpy as np
import yaml

WS = os.path.dirname(os.path.abspath(__file__))
CFG = os.path.join(WS, 'src/my_bot_bringup/config/cargo.yaml')
EDGE_TOL = 0.012        # m: mep anh lech mut LiDAR hon muc nay -> nghi thay mat ben


def load_cfg(path=CFG):
    with open(path) as f:
        return yaml.safe_load(f)


def calib_path(car):
    return os.path.join(os.path.dirname(CFG), f'camera_{car}.yaml')


def car_cfg(cfg, car, sim=False):
    """cfg rieng mot xe: camera_<xe>.yaml (tools/cam_calib.py + vi tri lap do tay)
    de len camera mac dinh. sim=True: giu camera mac dinh (camera ao dung so nay)."""
    out = dict(cfg)
    out['camera'] = dict(cfg['camera'])
    p = calib_path(car)
    if not sim and os.path.exists(p):
        with open(p) as f:
            out['camera'].update(yaml.safe_load(f) or {})
    return out


# ---------------------------------------------------------------------- LiDAR
def lidar_points(scans, lidar_x, cfg):
    """scans: [(angle_min, angle_increment, ranges)] -> diem (N,2) he base."""
    half = math.radians(cfg['lidar']['half_fov_deg'])
    if not scans:
        return np.zeros((0, 2))
    amin, inc, _ = scans[-1]
    n = len(scans[-1][2])
    same = [np.asarray(rr, float) for a0, i0, rr in scans
            if len(rr) == n and abs(a0 - amin) < 1e-6 and abs(i0 - inc) < 1e-9]
    # Xe dung yen: TRUNG VI tung tia qua cac vong quet (nhieu X2 ~1 cm/tia -> ~3 mm),
    # bo tia khong phan hoi (0/inf) truoc khi lay trung vi
    R = np.array(same)
    R[~np.isfinite(R) | (R <= 0.05)] = np.nan
    with np.errstate(all='ignore'), __import__('warnings').catch_warnings():
        __import__('warnings').simplefilter('ignore', RuntimeWarning)
        rr = np.nanmedian(R, axis=0)
        hits = np.sum(np.isfinite(R), axis=0)
    a = amin + np.arange(n) * inc
    a = np.arctan2(np.sin(a), np.cos(a))
    ok = (np.isfinite(rr) & (hits >= max(2, len(R) // 2)) & (rr < cfg['lidar']['max_range'])
          & (np.abs(a) < half))
    return np.c_[lidar_x + rr[ok] * np.cos(a[ok]), rr[ok] * np.sin(a[ok])]


def fit_face(P, rng=np.random.default_rng(0)):
    """Mat truoc hang tu diem LiDAR. Tra dict hoac raise ValueError."""
    if len(P) < 5:
        raise ValueError(f'LiDAR chi thay {len(P)} diem phia truoc - khong co hang?')
    # cum gan truc xe nhat: chia theo goc nhin, tach khi khoang cach nhay > 4 cm
    order = np.argsort(np.arctan2(P[:, 1], P[:, 0]))
    Q = P[order]
    near = np.argmin(np.abs(Q[:, 1]) + 0.3 * Q[:, 0])
    # gom diem lien ke trong 4 cm (lan tu diem gan truc)
    from scipy.spatial import cKDTree
    tree = cKDTree(Q)
    keep = np.zeros(len(Q), bool)
    keep[near] = True
    frontier = [near]
    while frontier:
        i = frontier.pop()
        for j in tree.query_ball_point(Q[i], 0.04):
            if not keep[j]:
                keep[j] = True
                frontier.append(j)
    C = Q[keep]
    if len(C) < 5:
        raise ValueError('cum diem truoc xe qua it')
    # RANSAC duong thang (nguong 8 mm) - mat truoc; diem mat ben (hop xoay) bi loai
    best, best_in = -1, None
    for _ in range(300):
        i, j = rng.choice(len(C), 2, replace=False)
        v = C[j] - C[i]
        L = np.linalg.norm(v)
        if L < 0.03:
            continue
        n = np.array([-v[1], v[0]]) / L
        if abs(n[0]) < 0.7:                    # mat lech > 45 do: khong phai mat truoc
            continue
        inl = np.abs((C - C[i]) @ n) < 0.008
        if inl.sum() > best:
            best, best_in = inl.sum(), inl
    if best_in is None:
        raise ValueError('khong khop duoc mat truoc hang')
    I = C[best_in]
    for _ in range(2):                         # khop lai bang binh phuong toi thieu
        m = I.mean(0)
        _, _, vt = np.linalg.svd(I - m)
        t = vt[0]                              # huong doc mat
        n = np.array([-t[1], t[0]])
        I = C[np.abs((C - m) @ n) < 0.008]
    m = I.mean(0)
    _, _, vt = np.linalg.svd(I - m)
    t = vt[0]
    n = np.array([-t[1], t[0]])
    if n[0] > 0:
        n = -n                                 # phap tuyen huong ve xe
    d = float(n @ m)
    s = (I - m) @ t
    ends = [m + s.min() * t, m + s.max() * t]
    ends.sort(key=lambda p: -p[1])             # [trai (y lon), phai]
    return {'n': n, 'd': d, 'mid': m, 'left': ends[0], 'right': ends[1],
            'yaw_deg': math.degrees(math.atan2(-n[1], -n[0])),
            'width': float(s.max() - s.min()), 'points': int(len(I)),
            'dist': float(-d),
            'rms_m': float(np.sqrt(np.mean(((I-m) @ n)**2)))}   # khoang cach tu goc base toi mat (doc phap tuyen)


# --------------------------------------------------------------------- camera
class Camera:
    def __init__(self, cfg):
        c = cfg['camera']
        self.K = np.array([[c['fx'], 0, c['cx']], [0, c['fy'], c['cy']], [0, 0, 1.0]])
        self.D = np.array(c['dist'], float)
        self.C = np.array([c['x'], c['y'], c['z']], float)
        # Goc xoay camera quanh truc dung so voi truc xe (do, + = quay sang TRAI) -
        # camera lap khong thang (tools/cam_extrinsic.py do, do 2026-10-15: xe 2 ~3-4 do).
        self.yaw = math.radians(float(c.get('yaw', 0.0)))
        self.cy_, self.sy_ = math.cos(self.yaw), math.sin(self.yaw)
        # Goc ngoc (do, + = nhin len) - tools/cam_extrinsic.py --height. Co pitch thi
        # chieu cao = mep tren -> SAN (khong can mep duoi: san bong phan chieu, mau chan
        # hop lan vao san lam mep duoi sai - do that 2026-10-15: +1.9 cm / mat nua hop).
        self.has_pitch = 'pitch' in c
        self.pitch = math.radians(float(c.get('pitch', 0.0)))
        self.cp_, self.sp_ = math.cos(self.pitch), math.sin(self.pitch)

    def project(self, p):
        """Diem 3D he base -> pixel (anh da khu meo)."""
        dx, dy, up = p[0] - self.C[0], p[1] - self.C[1], p[2] - self.C[2]
        fwd = dx * self.cy_ + dy * self.sy_          # xoay nguoc goc yaw ve he camera
        left = -dx * self.sy_ + dy * self.cy_
        fwd, up = fwd * self.cp_ + up * self.sp_, -fwd * self.sp_ + up * self.cp_   # nguoc pitch
        return np.array([self.K[0, 2] - self.K[0, 0] * left / fwd,
                         self.K[1, 2] - self.K[1, 1] * up / fwd])

    def ray(self, u, v):
        """Huong tia qua pixel (u, v) (anh khu meo) trong he base (khong chuan hoa)."""
        f, l, h = 1.0, -(u - self.K[0, 2]) / self.K[0, 0], -(v - self.K[1, 2]) / self.K[1, 1]
        f, h = f * self.cp_ - h * self.sp_, f * self.sp_ + h * self.cp_             # pitch
        return np.array([f * self.cy_ - l * self.sy_, f * self.sy_ + l * self.cy_, h])

    def ray_plane(self, u, v, n, d):
        """Tia qua pixel (u, v) (anh khu meo) cat mat phang dung n.(x,y) = d -> diem 3D."""
        dirv = self.ray(u, v)
        den = n @ dirv[:2]
        if abs(den) < 1e-6:
            raise ValueError('tia song song mat hang')
        t = (d - n @ self.C[:2]) / den
        return self.C + t * dirv


def segment(img):
    """Mat na hang: do bao hoa cao (Otsu) - nen trang/xam it mau."""
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    S = cv2.GaussianBlur(hsv[:, :, 1], (5, 5), 0)
    thr, mask = cv2.threshold(S, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    if thr < 25:                     # anh gan nhu khong co gi co mau
        mask[:] = 0
    k = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
    return mask


def front_rectangle_hint(img, uL, uR, v_seed, v_floor):
    """Closed image rectangle compatible with the LiDAR face and its floor.

    Used ONLY to constrain segmentation, never as a substitute for measured
    edges. Window/door rectangles above the cargo fail the floor/seed checks.
    A clipped box cannot create a full-frame-safe rectangle proposal.
    """
    H,W=img.shape[:2]
    lo,hi=sorted((float(uL),float(uR)));width=hi-lo
    gray=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    candidates=[]
    for g in (gray,cv2.createCLAHE(clipLimit=2.,tileGridSize=(8,8)).apply(gray)):
        edges=cv2.Canny(g,30,80)
        edges=cv2.morphologyEx(edges,cv2.MORPH_CLOSE,np.ones((3,3),np.uint8))
        contours,_=cv2.findContours(edges,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            perimeter=cv2.arcLength(contour,True)
            poly=cv2.approxPolyDP(contour,.02*perimeter,True)
            if len(poly)!=4 or not cv2.isContourConvex(poly):
                continue
            x,y,w,h=cv2.boundingRect(poly)
            area=cv2.contourArea(contour)
            tol=max(8.,.22*width)
            if (x<3 or y<3 or x+w>=W-2 or y+h>=H-2 or
                    abs(x-lo)>tol or abs(x+w-hi)>tol or
                    abs(y+h-v_floor)>max(8.,.22*width) or
                    h<max(12.,.4*width) or area/max(1.,w*h)<.7 or
                    not (x<.5*(lo+hi)<x+w and y<v_seed<y+h)):
                continue
            score=(abs(x-lo)+abs(x+w-hi)+abs(y+h-v_floor)+
                   5.*(1.-area/(w*h)))
            candidates.append((score,(x,y,w,h)))
    return min(candidates,key=lambda item:item[0])[1] if candidates else None


def visible_bottom_edge(gray, left, right, mask_bottom, floor_bottom):
    """Strong, coherent physical bottom edge, not a floor assumption/shadow.

    Require agreement of edges across >=60% of the face columns, within a
    narrow band around the segmented bottom and reasonably near the floor.
    """
    H,W=gray.shape
    if not (3.<mask_bottom<H-3 and abs(mask_bottom-floor_bottom)<16.):
        return None
    cols=np.linspace(left+.2*(right-left),left+.8*(right-left),15).astype(int)
    positions=[]
    for c in cols:
        profile=gray[:,max(c-1,0):min(c+2,W)].astype(float).mean(axis=1)
        profile=cv2.GaussianBlur(profile.reshape(-1,1),(1,3),0).ravel()
        gradient=np.abs(np.diff(profile))
        lo=max(1,int(mask_bottom)-4);hi=min(H-2,int(mask_bottom)+5)
        k=lo+int(np.argmax(gradient[lo:hi]))
        if gradient[k]>=6.:
            positions.append(k+.5)
    if len(positions)<9 or np.percentile(positions,90)-np.percentile(positions,10)>3.:
        return None
    return float(np.median(positions))


def grabcut_mask(img, uL, uR, v_ref, rectangle=None):
    """Mat na hop bang GrabCut, hat giong tu LiDAR: chac chan la hop = dai giua mat truoc
    o hang tia LiDAR; co the la hop = cot giua hai mut LiDAR; ngoai dai do la nen. Hoc mau
    hop/nen ngay tren anh -> chiu duoc rem sang, san bong, hop it mau (do that 2026-10-15:
    tach theo do bao hoa mat nua duoi hop nau, an bong phan chieu hop xanh)."""
    H, W = img.shape[:2]
    uL, uR = sorted((float(uL), float(uR)))
    w = max(uR - uL, 8.0)
    pad = max(25, int(0.4 * w))
    m = np.full((H, W), cv2.GC_BGD, np.uint8)
    m[:, max(int(uL) - pad, 0):min(int(uR) + pad, W)] = cv2.GC_PR_BGD
    m[:, max(int(uL) - 3, 0):min(int(uR) + 4, W)] = cv2.GC_PR_FGD
    vr = int(np.clip(v_ref, 6, H - 7))
    m[vr - 6:vr + 6, int(uL + 0.3 * w):int(uR - 0.3 * w) + 1] = cv2.GC_FGD
    if rectangle is not None:
        x,y,w,h=rectangle
        # Seed from the real image rectangle prevents the full-height probable
        # foreground strip from learning the door/window behind the cargo.
        allowed=np.zeros((H,W),bool)
        allowed[max(0,y-4):min(H,y+h+4),max(0,x-4):min(W,x+w+4)]=True
        m[~allowed]=cv2.GC_BGD
    bgd, fgd = np.zeros((1, 65)), np.zeros((1, 65))
    # Stable initialization: the same frame must not alternate between a
    # correct box mask and a door/window mask on successive evaluations.
    cv2.setRNGSeed(0)
    cv2.grabCut(img, m, None, bgd, fgd, 5, cv2.GC_INIT_WITH_MASK)
    fg = ((m == cv2.GC_FGD) | (m == cv2.GC_PR_FGD)).astype(np.uint8)
    k = np.ones((3, 3), np.uint8)
    fg = cv2.morphologyEx(cv2.morphologyEx(fg, cv2.MORPH_OPEN, k), cv2.MORPH_CLOSE, k)
    return fg > 0


def vertical_extent(mask, c0, c1, v0):
    """Hang tren/duoi cua hop: tu hang v0 di len (xuong), dung o hang dau tien hop chiem
    < 50% cac cot [c0, c1]. Chiu duoc vet ro hep (khe cua sang tren hop, bong duoi hop)."""
    H = mask.shape[0]
    c0, c1 = max(int(c0), 0), min(int(c1) + 1, mask.shape[1])
    frac = mask[:, c0:c1].mean(axis=1) if c1 > c0 else np.zeros(H)
    v0 = int(np.clip(v0, 0, H - 1))
    top = v0
    while top > 0 and frac[top - 1] >= 0.5:
        top -= 1
    bot = v0
    while bot < H - 1 and frac[bot + 1] >= 0.5:
        bot += 1
    return top - 0.5, bot + 0.5, top == 0, bot == H - 1


def runs_edges(row, c0):
    """Hang/cot nhi phan, diem c0 nam trong -> (mep dau, mep cuoi) toa do pixel (bien o)."""
    if not row[c0]:
        return None
    a = c0
    while a > 0 and row[a - 1]:
        a -= 1
    b = c0
    while b < len(row) - 1 and row[b + 1]:
        b += 1
    return a - 0.5, b + 0.5, a == 0 or b == len(row) - 1


def refine_edge(gray_row, u_pred, u_mask, side):
    """Hop xoay, camera thay ca mat ben: mep mat na la goc SAU. Goc truoc = vach
    doi do sang (mat truoc / mat ben) manh nhat BEN TRONG mat na, tu mep mat na
    toi qua mut LiDAR 12 px."""
    g = np.abs(np.diff(cv2.GaussianBlur(gray_row.astype(float).reshape(1, -1), (5, 1), 0).ravel()))
    if side == 'left':
        lo, hi = int(u_mask + 0.5) + 2, int(round(u_pred)) + 12
    else:
        lo, hi = int(round(u_pred)) - 12, int(u_mask - 0.5) - 2
    lo, hi = max(lo, 1), min(hi, len(g) - 1)
    if hi - lo < 2:
        return u_mask
    return lo + int(np.argmax(g[lo:hi])) + 0.5    # g[k] = |I(k+1) - I(k)| -> bien k+0.5


def edge_in_window(gray_row, lo, hi):
    """Vach doi do sang manh nhat (bien o) trong [lo, hi] cua mot hang anh xam."""
    g = np.abs(np.diff(cv2.GaussianBlur(gray_row.astype(float).reshape(1, -1), (5, 1), 0).ravel()))
    lo, hi = max(int(lo), 1), min(int(hi) + 1, len(g) - 1)
    if hi - lo < 2:
        return None
    return lo + int(np.argmax(g[lo:hi])) + 0.5


def front_top_edge(gray, mask, cols, va, vb):
    """Camera CAO hon mat tren hop: mep tren mat na la canh SAU cua mat tren. Mep tren
    that cua mat truoc = vach doi do sang (mat tren / mat truoc) manh nhat trong nua
    tren mat na. Tra hang (toa do bien o) hoac None neu khong co vach ro."""
    lo, hi = int(va) + 2, int(va + 0.5 * (vb - va))
    if hi - lo < 4:
        return None
    rows, strength = [], []
    for c in cols:
        prof = gray[lo:hi + 1, max(c - 1, 0):c + 2].astype(float).mean(axis=1)
        prof = np.convolve(prof, np.ones(3) / 3, mode='same')
        g = np.abs(np.diff(prof))[1:-1]
        if not len(g):
            continue
        k = int(np.argmax(g))
        rows.append(lo + 1 + k + 0.5)
        strength.append(g[k])
    if len(rows) < 5 or np.median(strength) < 4.0:
        return None
    return float(np.median(rows))


def measure(img_bgr, scans, cfg, lidar_x=None):
    """Tra dict ket qua (m) + anh ve chu thich. raise ValueError neu khong do duoc."""
    lx = cfg['lidar']['x'] if lidar_x is None else lidar_x
    face = fit_face(lidar_points(scans, lx, cfg))
    # Width from the two outermost ray centres is biased low by up to one
    # angular bin at EACH box edge. Add half a bin per edge for the central
    # estimate; retain the raw span below so the correction stays auditable.
    beam_step = (face['dist'] * abs(math.tan(scans[-1][1]))
                 if 0 < abs(scans[-1][1]) < math.radians(5) else 0.0)
    lidar_width = face['width'] + beam_step
    cam = Camera(cfg)
    c = cfg['camera']
    h0, w0 = img_bgr.shape[:2]
    warn = []
    if (w0, h0) != (c['width'], c['height']):
        # Luong ESP32-CAM co san co the khac co anh luc hieu chuan (vd QVGA 320x240):
        # cung ti le 4:3 thi phong anh ve co hieu chuan (cung khung nhin, chi thua pixel).
        if abs(w0 / h0 - c['width'] / c['height']) > 0.01:
            raise ValueError(f'anh {w0}x{h0} khac ti le co hieu chuan {c["width"]}x{c["height"]} - '
                             'hieu chuan lai: tools/cam_calib.py')
        img_bgr = cv2.resize(img_bgr, (c['width'], c['height']), interpolation=cv2.INTER_LINEAR)
        warn.append(f'anh {w0}x{h0} phong len {c["width"]}x{c["height"]} (hieu chuan o co khac) - '
                    f'1 px anh goc ~{face["dist"] / c["fx"] * c["width"] / w0 * 1000:.1f} mm')
    und = cv2.undistort(img_bgr, cam.K, cam.D)
    H, W = und.shape[:2]
    n, d = face['n'], face['d']
    if abs(face['yaw_deg']) > 15:
        warn.append(f'mat hang lech {face["yaw_deg"]:+.1f} do')
    # diem giua mat truoc o do cao LiDAR -> pixel hat giong
    zl = cfg['lidar']['z']
    u_mid, v_ref = cam.project(np.r_[face['mid'], zl])
    uL, _ = cam.project(np.r_[face['left'], zl])
    uR, _ = cam.project(np.r_[face['right'], zl])
    ui = int(round(u_mid))
    if not (0 <= ui < W):
        raise ValueError('mat hang ngoai khung anh')
    # Hat giong o NUA do cao tia LiDAR: hop dung tren san va LiDAR cham no, nen chac
    # chan phu tu san toi tia -> diem giua do nam tren mat truoc ke ca hop thap.
    support_z = float(cfg.get('support_z', 0.0))  # known floor / conveyor deck
    _, v_seed = cam.project(np.r_[face['mid'], 0.5 * (zl + support_z)])
    _,floor_ref=cam.project(np.r_[face['mid'],support_z])
    rectangle=front_rectangle_hint(und,uL,uR,v_seed,floor_ref)
    if cfg.get('require_front_rectangle', False) and rectangle is None:
        raise ValueError('chua tach chac mat truoc kien dau hang doi; khong do gop kien phia sau')
    if cfg.get('segment', 'grabcut') == 'grabcut':
        mask = grabcut_mask(und, uL, uR, v_seed, rectangle=rectangle)
    else:
        mask = segment(und) > 0
    if cfg.get('require_front_rectangle', False):
        # Similar-coloured rear carton can be learned as foreground inside the
        # normal 4px GrabCut margin. In FIFO mode the independently observed
        # closed front-face rectangle is also a hard association boundary.
        x,y,rw,rh=rectangle
        allowed=np.zeros(mask.shape,bool)
        allowed[y:y+rh,x:x+rw]=True
        mask &= allowed
    col = mask[:, ui]
    vs = int(round(v_seed))
    if not col[min(max(vs, 0), H - 1)]:
        # do cao LiDAR co the duoi khung/ngoai mat na: lay diem mat na gan nhat tren cot
        ys = np.nonzero(col)[0]
        if not len(ys):
            raise ValueError('camera khong thay hang (mat na rong tai cot giua)')
        vs = int(ys[np.argmin(np.abs(ys - v_ref))])
    top0, bot0, t_top, t_bot = vertical_extent(mask, min(uL, uR), max(uL, uR), vs)
    touch_v = t_top or t_bot
    # mep trai/phai: trung vi tren cac hang 35-65% chieu cao mat
    rows = np.linspace(top0 + 0.35 * (bot0 - top0), top0 + 0.65 * (bot0 - top0), 15).astype(int)
    gray = cv2.cvtColor(und, cv2.COLOR_BGR2GRAY)
    # Mat ben trai/phai lo ra khi camera nam ngoai mat phang mat ben do (hop xoay,
    # xe lech ngang). t: huong doc mat truoc, sang trai.
    t = np.array([-n[1], n[0]])
    if t[1] < 0:
        t = -t
    see_left = (cam.C[:2] - face['left']) @ t > 0.005
    see_right = (cam.C[:2] - face['right']) @ (-t) > 0.005
    # Cua so cho mep anh quanh mut LiDAR (camera.yaw da bu -> mut LiDAR chieu dung len
    # anh). Mut LiDAR nam TRONG mat hop toi ~1 khoang tia (sp); tol = 2 cm. Mep mat na ra
    # ngoai cua so (nguoc sang lam mat na lem ra nen, bong, mat ben) -> lay vach doi do
    # sang manh nhat trong cua so. Do that 2026-10-15: lem +4 cm o hop 18.5 cm.
    sp = cam.K[0, 0] * abs(scans[-1][1])
    tol = max(6.0, 0.02 * cam.K[0, 0] / face['dist'])
    winL, winR = (uL - sp - tol, uL + tol), (uR - tol, uR + sp + tol)
    lefts, rights, touch_h, clamped = [], [], False, 0
    for r in rows:
        e = runs_edges(mask[r], ui)
        if e is None:
            continue
        a, b, tch = e
        touch_h |= tch
        if see_left and (uL - a) * face['dist'] / cam.K[0, 0] > EDGE_TOL / 3:
            a = refine_edge(gray[r], uL, a, 'left')
        if see_right and (b - uR) * face['dist'] / cam.K[0, 0] > EDGE_TOL / 3:
            b = refine_edge(gray[r], uR, b, 'right')
        if not winL[0] <= a <= winL[1]:
            a2 = edge_in_window(gray[r], *winL)
            if a2 is not None:
                a, clamped = a2, clamped + 1
        if not winR[0] <= b <= winR[1]:
            b2 = edge_in_window(gray[r], *winR)
            if b2 is not None:
                b, clamped = b2, clamped + 1
        lefts.append(a)
        rights.append(b)
    if clamped > len(rows) // 2:
        warn.append('mep mat na lech xa mut LiDAR (nguoc sang? bong?) - mep lay theo vach sang gan LiDAR')
    if len(lefts) < 5:
        raise ValueError('khong tim duoc mep trai/phai')
    ua, ub = float(np.median(lefts)), float(np.median(rights))
    # mep tren/duoi: hang dau tien hop chiem < 50% be rong (khong theo tung cot: vet ro hep)
    va, vb, touch_top, t_bot = vertical_extent(mask, ua + 1, ub - 1, vs)
    touch_v = touch_top or t_bot
    # JPEG lam nhoe kenh mau 1-2 px -> mat na no ra (do that: rong +5..9 mm). Tinh chinh
    # mep theo vach DO SANG (kenh xam, net hon) trong +-3 px quanh mep mat na.
    r0, r1 = int(va + 0.35 * (vb - va)), int(va + 0.65 * (vb - va))
    prof_h = gray[max(r0, 0):max(r1, r0 + 1), :].astype(float).mean(axis=0)
    ua = edge_in_window(prof_h, ua - 3, ua + 3) or ua
    ub = edge_in_window(prof_h, ub - 3, ub + 3) or ub
    # GrabCut sometimes calls the dark label on a blue box its left boundary.
    # That boundary sits well INSIDE the LiDAR endpoint. Search the strong outer
    # brightness edge near the endpoint before deciding the box is unmeasurable.
    if ua - uL > 4.0:
        outer = edge_in_window(prof_h, uL - 12, uL + 4)
        if outer is not None and outer < ua - 2:
            ua = outer
            warn.append('mep trai mat na bi cat; lay vach ngoai gan LiDAR')
    if uR - ub > 4.0:
        outer = edge_in_window(prof_h, uR - 4, uR + 12)
        if outer is not None and outer > ub + 2:
            ub = outer
            warn.append('mep phai mat na bi cat; lay vach ngoai gan LiDAR')
    um = 0.5 * (ua + ub)
    if cam.ray_plane(um, va, n, d)[2] < cam.C[2] - 0.005:
        # mep tren mat na thap hon camera -> camera nhin thay mat tren hop
        cols = np.linspace(ua + 0.3 * (ub - ua), ua + 0.7 * (ub - ua), 15).astype(int)
        vf = front_top_edge(gray, mask, cols, va, vb)
        if vf is not None:
            va = vf
            warn.append('camera cao hon hang: mep tren lay theo vach sang mat tren/mat truoc')
        else:
            warn.append('camera cao hon hang, khong thay vach mat tren/mat truoc - chieu cao co the LON hon that')
    c0, c1 = int(ua + 0.3 * (ub - ua)), int(ua + 0.7 * (ub - ua))
    prof_v = gray[:, max(c0, 0):max(c1, c0 + 1)].astype(float).mean(axis=1)
    va = edge_in_window(prof_v, va - 3, va + 3) or va
    vm = 0.5 * (va + vb)
    PL, PR = cam.ray_plane(ua, vm, n, d), cam.ray_plane(ub, vm, n, d)
    PT, PB = cam.ray_plane(um, va, n, d), cam.ray_plane(um, vb, n, d)
    width = float(np.linalg.norm(PL[:2] - PR[:2]))
    height_source='visible_edges'
    if cam.has_pitch:
        vb_floor = float(cam.project(np.r_[(PL[:2] + PR[:2]) / 2, support_z])[1])
        bottom=visible_bottom_edge(gray,ua,ub,vb,vb_floor)
        if bottom is not None:
            bottom_point=cam.ray_plane(um,bottom,n,d)
        else:
            bottom_point=None
        if bottom_point is not None and abs(bottom_point[2]-support_z)<=.03:
            # When BOTH physical edges are visible, use their difference. This
            # cancels a common camera-height/floor offset instead of treating
            # the old ground calibration as an exact observed box bottom.
            vb=float(bottom);PB=bottom_point
            height=float(PT[2]-PB[2])
            touch_v=touch_top or vb>=H-1
        else:
            height=float(PT[2]-support_z)
            height_source='ground_plane'
            touch_v=touch_top or vb_floor>=H-1
    else:
        height = float(PT[2] - PB[2])
    if touch_h or touch_v:
        # rong/cao hon khung hinh: so do bi cat, khong duoc phan loai
        raise ValueError('hang cham mep anh (rong/cao qua khung hinh) - khong do duoc; '
                         'dat hang xa hon (place.gap) hoac hang nho hon')
    if abs(PB[2]-support_z) > 0.015:
        warn.append(f'mep duoi anh o z={PB[2] * 100:+.1f} cm' + (' (khong dung - cao tinh tu mep tren toi san)'
                    if cam.has_pitch else ' (khong tren san? bong do? - chua co camera.pitch)'))
    if abs(width - lidar_width) > 0.03:
        warn.append(f'rong camera {width * 100:.1f} cm khac LiDAR {lidar_width * 100:.1f} cm')
    # anh chu thich
    vis = und.copy()
    cnts, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(vis, cnts, -1, (0, 255, 255), 1)
    vb_draw = vb_floor if cam.has_pitch and height_source=='ground_plane' else vb
    cv2.rectangle(vis, (int(ua), int(va)), (int(ub), int(vb_draw)), (0, 0, 255), 2)
    for u in (uL, uR):
        cv2.line(vis, (int(u), int(v_ref) - 8), (int(u), int(v_ref) + 8), (255, 0, 0), 2)
    cv2.putText(vis, f'rong {width * 100:.1f} cm  cao {height * 100:.1f} cm  Z {(-d - cam.C[0]) * 100:.1f} cm',
                (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    return {'width': width, 'height': height, 'z_cam': float(-d - cam.C[0]),
            'face_yaw_deg': face['yaw_deg'], 'lidar_width': lidar_width,
            'lidar_width_raw': face['width'], 'lidar_beam_step_m': beam_step,
            'lidar_points': face['points'], 'bottom_z': float(PB[2]),
            'face_rms_m': face['rms_m'], 'height_source': height_source,
            'rectangle_hint': list(rectangle) if rectangle is not None else None,
            # A camera mask edge far INSIDE the corresponding LiDAR endpoint means
            # GrabCut has cut off part of the box (seen on the blue box: 10.4 cm
            # instead of ~12 cm, yet all five frames could agree on the error).
            'lidar_edge_inset_px': max(0.0, ua - uL, uR - ub),
            'edges_px': [ua, ub, va, vb], 'face_n': n.tolist(), 'face_d': float(d),
            'warnings': warn}, vis


def estimate_yaw(img_bgr, scans, cfg, lidar_x=None):
    """Goc xoay camera (do, + = trai) tu MOT hop dat thang truoc xe, cao hon tia LiDAR:
    huong toi giua mat truoc theo LiDAR - huong toi giua hop trong anh (camera chua xoay).
    Mut LiDAR co co vao trong (khoang cach tia) deu hai ben nen diem giua khong lech."""
    lx = cfg['lidar']['x'] if lidar_x is None else lidar_x
    face = fit_face(lidar_points(scans, lx, cfg))
    c = cfg['camera']
    c0 = dict(cfg)
    c0['camera'] = dict(c, yaw=0.0)
    cam0 = Camera(c0)
    if (img_bgr.shape[1], img_bgr.shape[0]) != (c['width'], c['height']):
        img_bgr = cv2.resize(img_bgr, (c['width'], c['height']), interpolation=cv2.INTER_LINEAR)
    und = cv2.undistort(img_bgr, cam0.K, cam0.D)
    zl = cfg['lidar']['z']
    m = face['mid']
    beta_lid = math.atan2(m[1] - cam0.C[1], m[0] - cam0.C[0])
    yaw = float(c.get('yaw', 0.0))
    for _ in range(3):            # GrabCut gieo theo LiDAR chieu voi yaw dang co -> yaw moi
        ck = dict(cfg)
        ck['camera'] = dict(c, yaw=yaw)
        cam = Camera(ck)
        uL, _ = cam.project(np.r_[face['left'], zl])
        uR, _ = cam.project(np.r_[face['right'], zl])
        u0, v_seed = cam.project(np.r_[m, 0.5 * zl])
        mask = grabcut_mask(und, uL, uR, v_seed)
        us, vs = int(round(u0)), int(round(v_seed))
        if not (0 <= us < mask.shape[1]) or not mask[min(max(vs, 0), mask.shape[0] - 1), us]:
            raise ValueError('khong thay hop trong anh quanh huong LiDAR')
        top, bot, _, _ = vertical_extent(mask, min(uL, uR), max(uL, uR), vs)
        lefts, rights = [], []
        for r in np.linspace(top + 0.35 * (bot - top), top + 0.65 * (bot - top), 15).astype(int):
            e = runs_edges(mask[r], us)
            if e is not None and not e[2]:
                lefts.append(e[0])
                rights.append(e[1])
        if len(lefts) < 5:
            raise ValueError('khong tim duoc mep trai/phai hop')
        um = 0.5 * (float(np.median(lefts)) + float(np.median(rights)))
        yaw = math.degrees(beta_lid - math.atan(-(um - c['cx']) / c['fx']))
    return {'yaw_deg': yaw, 'face_yaw_deg': face['yaw_deg'],
            'dist': face['dist'], 'u_lidar': float(u0), 'u_cam': um, 'lidar_width': face['width']}


def classify(cfg, width, height):
    for c in cfg['classes']:
        if width <= c['max_w'] and height <= c['max_h']:
            return c
    return None


# ------------------------------------------------------------- do mot lan (CLI)
def main():
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, LaserScan
    args = sys.argv[1:]
    car = args[0] if args and not args[0].startswith('--') else 'agv1'
    out = args[args.index('--save') + 1] if '--save' in args else None
    cfg = car_cfg(load_cfg(), car, sim='--sim' in args)
    rclpy.init()
    node = rclpy.create_node('cargo_measure_once')
    scans, img = [], {}
    node.create_subscription(LaserScan, f'/{car}/scan',
                             lambda m: scans.append((m.angle_min, m.angle_increment, list(m.ranges))),
                             qos_profile_sensor_data)
    node.create_subscription(Image, f'/{car}/camera/image_raw', lambda m: img.__setitem__('m', m), 2)
    t0 = time.time()
    while time.time() - t0 < 10 and (len(scans) < cfg['lidar']['scans'] or 'm' not in img):
        rclpy.spin_once(node, timeout_sec=0.1)
    if 'm' not in img or not scans:
        sys.exit(f'Khong nhan duoc anh /{car}/camera/image_raw hoac /{car}/scan')
    m = img['m']
    frame = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, 3)
    lx = float(os.environ.get('LIDAR_X', cfg['lidar']['x']))
    res, vis = measure(frame, scans[-cfg['lidar']['scans']:], cfg, lx)
    c = classify(cfg, res['width'], res['height'])
    print(f'{car}: rong {res["width"] * 100:.1f} cm, cao {res["height"] * 100:.1f} cm '
          f'(Z camera {res["z_cam"] * 100:.1f} cm, mat lech {res["face_yaw_deg"]:+.1f} do, '
          f'LiDAR rong {res["lidar_width"] * 100:.1f} cm) -> '
          + (f'loai "{c["name"]}" -> o (1,{c["col"]})' if c else 'KHONG khop loai nao'))
    for w in res['warnings']:
        print('  canh bao:', w)
    if out:
        cv2.imwrite(out, vis)
        print('  anh:', out)


if __name__ == '__main__':
    main()
