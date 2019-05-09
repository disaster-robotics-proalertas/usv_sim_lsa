import math
from geometry_msgs.msg import Point

def findDistance(x0, y0, x1, y1):
    return math.sqrt(pow(x1-x0, 2) + pow(y1-y0, 2))

def angleToPoint(lat, lon):
    tmp = Point()
    tmp.x = lat
    tmp.y = lon

    return tmp

def adjustFrame(sensor):
    if sensor > 180:
        sensor -= 360
    if sensor < -180:
        sensor += 360
    return sensor

def projection2dMod(lat, lon, a, b, a_p, b_p):
    if a == 0:
        a = 0.00000001
    if b == 0:
        b = 0.00000001

    a_aux = -1/a
    b_aux = -a_aux * lat + lon

    latProj = (b_aux - b_p) / (a_p - a_aux)
    lonProj = a_p * latProj + b_p

    return angleToPoint(latProj, lonProj)

def projection2d(lat, lon, a, b):
    if a == 0:
        a = 0.00000001
    if b == 0:
        b = 0.00000001

    a_aux = -1/a
    b_aux = -a_aux * lat + lon

    latProj = (b_aux - b) / (a - a_aux)
    lonProj = a * latProj + b

    return angleToPoint(latProj, lonProj)

def tackPoints(current, target, tackingAngle, tackingWidenessRate, _heeling, sp):

    x0 = current.x
    y0 = current.y
    x = target.x
    y = target.y
#    x0 = current.y
#    y0 = current.x
#    x = target.y
#    y = target.x
    _tackingAngle = tackingAngle

    initialDistance = findDistance(x0, y0, x, y)
    tackingWideness = tackingWidenessRate * initialDistance

    a_A = (y - y0) / (x - x0)
    b_A = y0 - (a_A * x0)

    if (_heeling - sp < 0):
        _tackingAngle = -_tackingAngle

    thetaAB = _tackingAngle
    thetaAB = adjustFrame(thetaAB)
    tan_thetaAB = math.tan(math.radians(thetaAB))

    a_B = (a_A - tan_thetaAB) / (tan_thetaAB * a_A + 1)
    b_B = y0 - (a_B * x0)

    d = initialDistance
    d_p0 = 9999999

    latBord = x
    lonBord = y

    latIni = x0
    lonIni = y0
    latFim = x
    lonFim = y
    a = tackingWideness
    pontoProj = Point()
    count = 0

    while abs(d - a) > 1:
        latMedia = (latFim + latIni) / 2
        lonMedia = (lonFim + lonIni) / 2
        
#        if count == 0:
#            print(latMedia)    
#            print(lonMedia)    

        pontoProj = projection2dMod(latMedia, lonMedia, a_A, b_A, a_B, b_B)

        d = findDistance(latMedia, lonMedia, pontoProj.x, pontoProj.y)

        if d > a:
            latFim = latMedia
            lonFim = lonMedia
        elif d < a:
            latIni = latMedia
            lonIni = lonMedia

        latBord = latMedia
        lonBord = lonMedia
        count += 1
        

    ponto_projetado = angleToPoint(latBord, lonBord)

    latProj = pontoProj.x
    lonProj = pontoProj.y

    p0m = projection2d(latProj, lonProj, a_A, b_A)
    d_p0 = findDistance(p0m.x, p0m.y, x0, y0)

    TackingPointsVector = [angleToPoint(latProj, lonProj)]

    b_linha1 = (-a_A * TackingPointsVector[0].x + TackingPointsVector[0].y)
    
    if b_linha1 > b_A:
        b_linha2 = b_A - (b_linha1 - b_A)
    else:
        b_linha2 = b_A + (b_A - b_linha1)

    num_pontos = math.floor(findDistance(x0, y0, x, y) / d_p0)
    
    delta_x = p0m.x - x0
    delta_y = p0m.y - y0

    p0l1 = projection2d(p0m.x, p0m.y, a_A, b_linha1)
    p0l2 = projection2d(p0m.x, p0m.y, a_A, b_linha2)
    bom = 1
    ruim = 1

    for z in range(1, int(num_pontos)):
        delta_x_temp = z * delta_x
        delta_y_temp = z * delta_y

        if z % 2 == 0:
            controle1 = TackingPointsVector[z-1]
            lat_temp = p0l1.x + delta_x_temp
            lon_temp = p0l1.y + delta_y_temp
            controle_loc = angleToPoint(lat_temp, lon_temp)
            TackingPointsVector.append(controle_loc)
            bom = z
        else:
            controle1 = TackingPointsVector[z-1]
            aux = abs((_heeling - abs(adjustFrame(sp))))
            delta_xaux = aux * delta_x / 31
            delta_yaux = aux * delta_y / 31
            lat_temp = p0l2.x + delta_x_temp - delta_xaux
            lon_temp = p0l2.y + delta_y_temp - delta_yaux
            controle_loc = angleToPoint(lat_temp, lon_temp)
            TackingPointsVector.append(controle_loc)
            ruim = z

    numberOfTackingPoints = len(TackingPointsVector)
    d_p0_pu = findDistance(TackingPointsVector[numberOfTackingPoints-1].x, TackingPointsVector[numberOfTackingPoints-1].y, current.x, current.y)
    d_p0_pd = findDistance(x0, y0, x, y)

#    if(d_p0_pu > d_p0_pd):
#        TackingPointsVector[numberOfTackingPoints-1] = target 
#    else:
    TackingPointsVector.append(target)

    return TackingPointsVector

