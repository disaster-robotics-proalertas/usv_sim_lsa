# This script is use to convert the shaders found
# within the resources/shaders directory to the native
# .inl format for compiling into the osgOcean source.
# After conversion, it will copy the .inl files into the
# include/osgOcean/shaders/ directory.

import os
import time

###############################################

def readCopyright():
    copyrightFile = open("copyright_notice.txt","r")
    copyright = copyrightFile.readlines()
    copyrightFile.close()
    return copyright

###############################################

def shaderVarName( shaderName ):
    varName = shaderName.replace(".","_")
    return varName

###############################################

def isCurrent( shader, header ):
    if os.path.isfile( header ):
        shaderMod = os.path.getmtime(shader)
        headerMod = os.path.getmtime(header)
        if shaderMod < headerMod:
            return True
    return False

###############################################

def createInlShader( shaderFile, shaderVar, headerFile ):  
    file = open(shaderFile,'r')
    lines = file.readlines()
    file.close()
   
    oFile = open( headerFile, "w")
    
    for line in copyright:
        oFile.write(line)

    oFile.write("\nstatic const char " + shaderVar +"[] =")

    for line in lines:
        newLine = line.replace("\n","").replace("\r","")
        oFile.write('\n\t"'+newLine+'\\n"')

    oFile.write(";\n");
    oFile.flush()
    oFile.close()

##############################################

print("\nThis script is used to convert the osgOcean shaders")
print("found within the resources/shaders directory to the")
print("native .inl format for compiling into the osgOcean")
print("source.\n")
print("Once converted, the .inl files will be copied into the")
print("include/osgOcean/shaders/ directory overwriting")
print("existing files.\n")

confirm = raw_input("Continue? [y/n]: ")

if confirm == 'n' or confirm == 'N':
    exit()
        
shaderPath = "../resources/shaders/"
headerPath = "../include/osgOcean/shaders/"

shaderList = os.listdir( shaderPath )

skipped = 0
created = 0

print("\nProcessing shader files")
print("--------------------------------\n")

copyright = readCopyright()

for shader in shaderList:
    if shader.find("osgOcean_") > -1:
        if shader.rfind(".vert") > -1 or shader.rfind(".frag") > -1:
            sVar = shaderVarName(shader)
            hName = sVar + ".inl"
            hFile = headerPath + hName
            sFile = shaderPath + shader

            if isCurrent(sFile,hFile) == True:
                skipped += 1
                print("[skipped] " + sVar)
            else:
                createInlShader( sFile, sVar, hFile )
                created += 1
                print("[CREATED] " + sVar )
                
print("\n--------------------------------")
print(str(created)+"\tCreated")
print(str(skipped)+"\tUp to date")
print(str(skipped+created)+"\tTotal")
print("--------------------------------")    



