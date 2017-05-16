SET(HEADER_PATH ${osgOcean_SOURCE_DIR}/include/OceanExample )
SET(LIB_HEADER_PATH ${osgOcean_SOURCE_DIR}/include )

SET( SKYBOX_TEXTURES_1
  ${osgOcean_SOURCE_DIR}/resources/textures/sky_clear/down.png
  ${osgOcean_SOURCE_DIR}/resources/textures/sky_clear/east.png
  ${osgOcean_SOURCE_DIR}/resources/textures/sky_clear/north.png
  ${osgOcean_SOURCE_DIR}/resources/textures/sky_clear/south.png
  ${osgOcean_SOURCE_DIR}/resources/textures/sky_clear/up.png
  ${osgOcean_SOURCE_DIR}/resources/textures/sky_clear/west.png
)

SET( SKYBOX_TEXTURES_2
  ${osgOcean_SOURCE_DIR}/resources/textures/sky_dusk/down.png
  ${osgOcean_SOURCE_DIR}/resources/textures/sky_dusk/east.png
  ${osgOcean_SOURCE_DIR}/resources/textures/sky_dusk/north.png
  ${osgOcean_SOURCE_DIR}/resources/textures/sky_dusk/south.png
  ${osgOcean_SOURCE_DIR}/resources/textures/sky_dusk/up.png
  ${osgOcean_SOURCE_DIR}/resources/textures/sky_dusk/west.png
)

SET( SKYBOX_TEXTURES_3
  ${osgOcean_SOURCE_DIR}/resources/textures/sky_fair_cloudy/down.png
  ${osgOcean_SOURCE_DIR}/resources/textures/sky_fair_cloudy/east.png
  ${osgOcean_SOURCE_DIR}/resources/textures/sky_fair_cloudy/north.png
  ${osgOcean_SOURCE_DIR}/resources/textures/sky_fair_cloudy/south.png
  ${osgOcean_SOURCE_DIR}/resources/textures/sky_fair_cloudy/up.png
  ${osgOcean_SOURCE_DIR}/resources/textures/sky_fair_cloudy/west.png
)

SET( ISLAND_RESOURCES
  ${osgOcean_SOURCE_DIR}/resources/island/islands.ive
)

SET( BOAT_RESOURCES 
  ${osgOcean_SOURCE_DIR}/resources/boat/boat.3ds
)

SET( EXAMPLE_SHADERS
  ${osgOcean_SOURCE_DIR}/resources/shaders/terrain.vert
  ${osgOcean_SOURCE_DIR}/resources/shaders/terrain.frag
  ${osgOcean_SOURCE_DIR}/resources/shaders/terrain_lispsm.vert
  ${osgOcean_SOURCE_DIR}/resources/shaders/terrain_lispsm.frag
  ${osgOcean_SOURCE_DIR}/resources/shaders/skydome.vert
  ${osgOcean_SOURCE_DIR}/resources/shaders/skydome.frag
)

# Adds shaders to the "Shaders" group in visual studio
SOURCE_GROUP( "Shaders" FILES ${EXAMPLE_SHADERS} )

INCLUDE_DIRECTORIES(
   ${LIB_HEADER_PATH}
   ${HEADER_PATH}
)

ADD_EXECUTABLE(

  oceanExample

  application.cpp
  Scene.cpp
  SkyDome.cpp
  SphereSegment.cpp
  

  ${HEADER_PATH}/SceneEventHandler.h
  ${HEADER_PATH}/Scene.h
  ${HEADER_PATH}/ScopedTimer.h
  ${HEADER_PATH}/SkyDome.h
  ${HEADER_PATH}/SphereSegment.h
  ${HEADER_PATH}/TextHUD.h

  ${EXAMPLE_SHADERS}
)

TARGET_LINK_LIBRARIES(
  

  oceanExample

  osgOcean

  debug ${OSG_LIBRARY_DEBUG}
  optimized ${OSG_LIBRARY}

  debug ${OSGGA_LIBRARY_DEBUG}
  optimized ${OSGGA_LIBRARY}

  debug ${OSGVIEWER_LIBRARY_DEBUG}
  optimized ${OSGVIEWER_LIBRARY}

  debug ${OSGDB_LIBRARY_DEBUG}
  optimized ${OSGDB_LIBRARY}

  debug ${OSGTEXT_LIBRARY_DEBUG}
  optimized ${OSGTEXT_LIBRARY}

  debug ${OSGSHADOW_LIBRARY_DEBUG}
  optimized ${OSGSHADOW_LIBRARY}
)

INSTALL(
   TARGETS oceanExample
   RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
)

INSTALL(
  FILES ${SKYBOX_TEXTURES_1}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/bin/resources/textures/sky_clear
)

INSTALL(
  FILES ${SKYBOX_TEXTURES_2}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/bin/resources/textures/sky_dusk
)

INSTALL(
  FILES ${SKYBOX_TEXTURES_3}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/bin/resources/textures/sky_fair_cloudy
)

INSTALL(
  FILES ${EXAMPLE_SHADERS}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/bin/resources/shaders
)

INSTALL(
  FILES ${ISLAND_RESOURCES}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/bin/resources/island
)

INSTALL(
  FILES ${BOAT_RESOURCES}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/bin/resources/boat
)





