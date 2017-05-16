@echo on

geometryop %* dectest00.osg
geometryop %* dectest01.osg
geometryop %* dectest02.osg

geometryop %* dectest10.osg
geometryop %* dectest11.osg
geometryop %* dectest12.osg

rem degenerate triangle
geometryop %* dectest13.osg
rem redundant triangle
geometryop %* dectest14.osg

rem solidworks door and knob
geometryop %* dectest20.ive
rem knob only
geometryop %* dectest21.osg
rem simple case demonstrating an issue with ReducerOp
geometryop %* dectest22.osg

rem test grouping ability, don't reduce past groups of nearly coplanar tris.
geometryop %* dectest30.osg
geometryop %* dectest31.osg
geometryop %* dectest32.osg

geometryop %* pliers-big.osg
