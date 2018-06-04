def tackPoints(current, target, tackingAngle, tackingWidenessRate):
    x0 = current.x
    y0 = current.y
    x1 = target.x
    y1 = target.y

    initialDistance = findDistance(current, target)
    tackingWideness = tackingWidenessRate * initialDistance

    a_A = (y - y0) / (x - x0)
    b_A = y0 - (a_A * x0)

    if (_heeling - _heading < 0):
        tackingAngle = -tackingAngle

    thetaAB = tackingAngle
    thetaAB = adjustFrame(thetaAB)

    tan_thetaAB = (float)tan((thetaAB) * (PI / 180))

    a_B = (a_A - tan_thetaAB) / (tan_thetaAB * a_A + 1)
    b_B = y0 - (a_B * x0)

    d = initialDistance
    d_p0 = 9999999

    latBord = x
    lonBord = y

    latProj = x
    lonProj = y

    latIni = x0
    lonIni = y0
    latFim = x
    lonFim = y

    while fabs(d - tackingWideness) > 1:
        latMedia = (latFim + latIni) / 2
        lonMedia = (lonFim + lonIni) / 2

        pontoProj = projection2Mod(latMedia, lonMedia, a_A, b_A, a_B, b_B)

        d = float(findDistance(latMedia, lonMedia, pontoProj.x, pontoProj.y))

        if d > tackingWidness:
            latFim = latMedia
            lonFim = lonMedia
        elif d < tackingWideness:
            latIni = latMedia
            lonIni = lonMedia

        latBord = latMedia
        lonBord = lonMedia

    ponto_projetado = angleToLocation(latBord, lonBord)

    latProj = pontoProj.x
    lonProj = pontoProj.y

    p0m = projection2d(latProj, lonProj, a_A, b_A)
    d_p0 = (float)findDistance(p0m.x, p0m.y, x0, y0)

    TackingPointsVector.append(angleToLocation(latProj, lonProj))

    b_linha1 = (-a_A * TackingPointsVector.at(0).x + TackingPointsVector.at(0).y)
    
    if b_linha1 > b_A:
        b_linha2 = b_A - (b_linha1 - b_A)
    else:
        b_linha2 = b_A + (b_A - b_linha1)

    num_pontos = (int)floor((float)FindDistance(x0, y0, x, y) / d_p0)
    
    delta_x = p0m.x - x0
    delta_y = p0m.y - y0

    p0l1 = projection2d(p0m.x, p0m.y, a_A, b_linha1)
    p0l2 = projection2d(p0m.x, p0m.y, a_A, b_linha2)
    bom = 1
    ruim = 1

    for z in range(1, num_pontos):
        delta_x_temp = z * delta_x
        delta_y_temp = z * delta_y

        if z % 2 == 0:
            controle1 = TackingPointsVector.at(z-1)
            lat_temp = (float)p0l1.x + delta_x_temp
            lon_temp = (float)p0l1.y + delta_y_temp
            controle_loc = angleToLocation(lat_temp, lon_temp)
            TackingPointsVector.append(controle_loc)
            bom = z
        else:
            controle1 = TackingPointsVector.at(z-1)
            aux = fabs((_heeling - fabs(adjustFrame(_sp_))))
            delta_xaux = aux * delta_x / 31
            delta_yaux = aux * delta_y / 31
            lat_temp = (float)p0l1.x + delta_x_temp - delta_xaux
            lon_temp = (float)p0l1.y + delta_y_temp - delta_yaux
            controle_loc = angleToLocation(lat_temp, lon_temp)
            TackingPointsVector.append(controle_loc)
            ruim = z

    numberOfTackingPoints = TackingPointsVector.size()





























