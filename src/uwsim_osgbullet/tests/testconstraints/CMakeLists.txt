set( _exeName testconstraints )

_osgBulletMakeInteractionExe( ${_exeName}
    testconstraints.cpp
    ctest.cpp
    ctest.h
)

set( _constraintTypes
    LinearSpring
    AngleSpring
    LinearAngleSpring
    Fixed
    Planar
    Box
    Slider
    TwistSlider
    Hinge
    Cardan
    BallAndSocket
    Ragdoll
    WheelSuspension
)

foreach( _constraint ${_constraintTypes} )
    add_test(
        NAME ${_constraint}Constraint
        COMMAND ${_exeName} --ctest ${_constraint}
    )
endforeach()
