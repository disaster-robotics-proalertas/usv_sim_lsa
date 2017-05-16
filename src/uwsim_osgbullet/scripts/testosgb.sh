#/bin/sh

echo
echo Turn-on tests. Does it work at all?
echo Run BasicDemo, RobotArm.
echo
BasicDemo
robot


echo
echo
echo
echo Decimation tests.
echo First, test the simpifier.
echo
osgbpp cow.osg --convexTM --overall
osgbpp cow.osg --convexTM --overall --simplify .2
echo
echo Decimation tests.
echo Test the decimator
echo
osgbpp cow.osg --convexTM --overall --decPercent .2
echo
echo Decimation tests.
echo Test the Vertex Aggregator.
echo
osgbpp cow.osg --convexTM --overall --aggMaxVerts 50
echo
echo Decimation tests, this time with the pliers.
echo First, test the simpifier.
echo
osgbpp pliers-big.osg --convexTM --overall
osgbpp pliers-big.osg --convexTM --overall --simplify .2
echo
echo Decimation tests.
echo Test the decimator
echo
osgbpp pliers-big.osg --convexTM --overall --decPercent .2
echo
echo Decimation tests.
echo Test the Vertex Aggregator.
echo
osgbpp pliers-big.osg --convexTM --overall --aggMaxVerts 50


echo
echo
echo
echo Testing the tetra.
echo All should behave normally.
echo
osgbpp tetra.osg --convexHull
osgbpp tetra.osg --convexTM

echo
echo
echo
echo Testing the concave object.
echo All should behave normally.
echo
osgbpp concave.osg --convexHull
osgbpp concave.osg --convexHull --overall
osgbpp concave.osg --triMesh
osgbpp concave.osg --box
osgbpp concave.osg --box --overall

echo
echo
echo
echo Testing the cow.
echo All should behave normally.
echo
osgbpp cow.osg --convexHull --overall
osgbpp cow.osg --triMesh --overall

echo
echo
echo
echo Center of mass tests.
echo First test \(cube\) should behave normally.
echo Next four cubes should behave as if COM is outside lower left corner.
echo
osgbpp offcube.osg --box
osgbpp offcube.osg --box --com 0,0,0
osgbpp offcube.osg --triMesh --com 0,0,0
osgbpp offcube.osg --convexTM --com 0,0,0
osgbpp offcube.osg --convexHull --com 0,0,0
echo
echo Center of mass tests.
echo Concave object should behave as if COM is outside to the right.
echo Cow should behave as if COM is outside to the right-back.
echo
osgbpp concave.osg --box --com 5,0,0
osgbpp cow.osg --convexHull --overall --com 2,0,0


echo
echo Decimation combined with center of mass.
echo
osgbpp cow.osg --convexTM --overall --simplify .2 --com 4,-.5,0
osgbpp cow.osg --convexTM --overall --decPercent .2  --com 4,-.5,0
osgbpp cow.osg --convexTM --overall --aggMaxVerts 50  --com 4,-.5,0
