#!/ usr/bin/python
# -*- coding:utf-8 -*-
# 1、https://blog.csdn.net/normalstudent/article/details/82223350
# 2、https://blog.csdn.net/weixin_42015762/article/details/105835752

import math
home_lat = 15.1269321    # Somewhere random
home_lon = 1.6624301     # Somewhere random
home_alt = 163000       # Somewhere random
def LatLon2XY(latitude, longitude):
    a = 6378137.0
    # b = 6356752.3142
    # c = 6399593.6258
    # alpha = 1 / 298.257223563
    e2 = 0.0066943799013
    # epep = 0.00673949674227
    #将经纬度转换为弧度
    latitude2Rad = (math.pi / 180.0) * latitude
    beltNo = int((longitude + 1.5) / 3.0) #计算3度带投影度带号
    L = beltNo * 3 #计算中央经线
    l0 = longitude - L #经差
    tsin = math.sin(latitude2Rad)
    tcos = math.cos(latitude2Rad)
    t = math.tan(latitude2Rad)
    m = (math.pi / 180.0) * l0 * tcos
    et2 = e2 * pow(tcos, 2)
    et3 = e2 * pow(tsin, 2)
    X = 111132.9558 * latitude - 16038.6496 * math.sin(2 * latitude2Rad) + 16.8607 * math.sin(
        4 * latitude2Rad) - 0.0220 * math.sin(6 * latitude2Rad)
    N = a / math.sqrt(1 - et3)

    x = X + N * t * (0.5 * pow(m, 2) + (5.0 - pow(t, 2) + 9.0 * et2 + 4 * pow(et2, 2)) * pow(m, 4) / 24.0 + (
    61.0 - 58.0 * pow(t, 2) + pow(t, 4)) * pow(m, 6) / 720.0)
    y = 500000 + N * (m + (1.0 - pow(t, 2) + et2) * pow(m, 3) / 6.0 + (
    5.0 - 18.0 * pow(t, 2) + pow(t, 4) + 14.0 * et2 - 58.0 * et2 * pow(t, 2)) * pow(m, 5) / 120.0)

    return y, x, L #X,Y轴反了！反了！

def XY2LatLon(X, Y, L0): # L0中央经线

    iPI = 0.0174532925199433
    a = 6378137.0
    f= 0.00335281006247
    ZoneWide = 3 #按3度带进行投影

    ProjNo = int(X / 1000000)
    L0 = L0 * iPI
    X0 = ProjNo * 1000000 + 500000
    Y0 = 0
    xval = X - X0
    yval = Y - Y0

    e2 = 2 * f - f * f #第一偏心率平方
    e1 = (1.0 - math.sqrt(1 - e2)) / (1.0 + math.sqrt(1 - e2))
    ee = e2 / (1 - e2) #第二偏心率平方

    M = yval
    u = M / (a * (1 - e2 / 4 - 3 * e2 * e2 / 64 - 5 * e2 * e2 * e2 / 256))

    fai = u \
          + (3 * e1 / 2 - 27 * e1 * e1 * e1 / 32) * math.sin(2 * u) \
          + (21 * e1 * e1 / 16 - 55 * e1 * e1 * e1 * e1 / 32) * math.sin(4 * u) \
          + (151 * e1 * e1 * e1 / 96) * math.sin(6 * u)\
          + (1097 * e1 * e1 * e1 * e1 / 512) * math.sin(8 * u)
    C = ee * math.cos(fai) * math.cos(fai)
    T = math.tan(fai) * math.tan(fai)
    NN = a / math.sqrt(1.0 - e2 * math.sin(fai) * math.sin(fai))
    R = a * (1 - e2) / math.sqrt(
        (1 - e2 * math.sin(fai) * math.sin(fai)) * (1 - e2 * math.sin(fai) * math.sin(fai)) * (1 - e2 * math.sin(fai) * math.sin(fai)))
    D = xval / NN

    #计算经纬度（弧度单位的经纬度）
    longitude1 = L0 + (D - (1 + 2 * T + C) * D * D * D / 6 + (
    5 - 2 * C + 28 * T - 3 * C * C + 8 * ee + 24 * T * T) * D * D * D * D * D / 120) / math.cos(fai)
    latitude1 = fai - (NN * math.tan(fai) / R) * (
    D * D / 2 - (5 + 3 * T + 10 * C - 4 * C * C - 9 * ee) * D * D * D * D / 24 + (
    61 + 90 * T + 298 * C + 45 * T * T - 256 * ee - 3 * C * C) * D * D * D * D * D * D / 720)

    #换换为deg
    longitude = longitude1 / iPI
    latitude = latitude1 / iPI

    return latitude, longitude

def targetXY2LB(X, Y):
    HomeXY = LatLon2XY(home_lat,home_lon)
    targetXY = (HomeXY[0] + X, HomeXY[1] + Y)
    return XY2LatLon(targetXY[0], targetXY[1], HomeXY[2])

def relativeLatLon2XY(L1, B1, L2, B2):
    return(math.sqrt((LatLon2XY(L1,B1)[0]-LatLon2XY(L2,B2)[0])**2 + (LatLon2XY(L1,B1)[1]-LatLon2XY(L2,B2)[1])**2))

# print (LatLon2XY(31.6122917,118.4459928)[0])
# print (LatLon2XY(31.6113473,118.4460336))
print (math.sqrt((LatLon2XY(15.1268671,1.6625165)[0]-LatLon2XY(15.1282039,1.6805220)[0])**2+(LatLon2XY(15.1268671,1.6625165)[1]-LatLon2XY(15.1282039,1.6805220)[1])**2))
print (LatLon2XY(31.6113473,118.4460336))
print (XY2LatLon(637220.0986797615,3499665.031476644,LatLon2XY(31.6113473,118.4460336)[2]))
print (targetXY2LB(1,0))
print (relativeLatLon2XY(home_lat, home_lon, targetXY2LB(1,0)[0], targetXY2LB(1,0)[1]))