SET(HEADER_PATH ${PROJECT_SOURCE_DIR}/include/osgOcean )

SET( SHADER_FILES
  ${PROJECT_SOURCE_DIR}/resources/shaders/osgOcean_ocean_surface.frag
  ${PROJECT_SOURCE_DIR}/resources/shaders/osgOcean_ocean_surface.vert
  ${PROJECT_SOURCE_DIR}/resources/shaders/osgOcean_ocean_surface_vbo.vert

  ${PROJECT_SOURCE_DIR}/resources/shaders/osgOcean_godrays.vert
  ${PROJECT_SOURCE_DIR}/resources/shaders/osgOcean_godrays.frag
  ${PROJECT_SOURCE_DIR}/resources/shaders/osgOcean_godray_screen_blend.vert
  ${PROJECT_SOURCE_DIR}/resources/shaders/osgOcean_godray_screen_blend.frag
  ${PROJECT_SOURCE_DIR}/resources/shaders/osgOcean_godray_glare.vert
  ${PROJECT_SOURCE_DIR}/resources/shaders/osgOcean_godray_glare.frag

  ${PROJECT_SOURCE_DIR}/resources/shaders/osgOcean_silt_quads.vert
  ${PROJECT_SOURCE_DIR}/resources/shaders/osgOcean_silt_quads.frag
  ${PROJECT_SOURCE_DIR}/resources/shaders/osgOcean_silt_points.vert
  ${PROJECT_SOURCE_DIR}/resources/shaders/osgOcean_silt_points.frag

  ${PROJECT_SOURCE_DIR}/resources/shaders/osgOcean_streak.vert
  ${PROJECT_SOURCE_DIR}/resources/shaders/osgOcean_streak.frag
  ${PROJECT_SOURCE_DIR}/resources/shaders/osgOcean_glare_composite.vert
  ${PROJECT_SOURCE_DIR}/resources/shaders/osgOcean_glare_composite.frag

  ${PROJECT_SOURCE_DIR}/resources/shaders/osgOcean_downsample_glare.frag
  ${PROJECT_SOURCE_DIR}/resources/shaders/osgOcean_downsample.vert
  ${PROJECT_SOURCE_DIR}/resources/shaders/osgOcean_downsample.frag

  ${PROJECT_SOURCE_DIR}/resources/shaders/osgOcean_gaussian.vert
  ${PROJECT_SOURCE_DIR}/resources/shaders/osgOcean_gaussian1.frag
  ${PROJECT_SOURCE_DIR}/resources/shaders/osgOcean_gaussian2.frag
  ${PROJECT_SOURCE_DIR}/resources/shaders/osgOcean_dof_combiner.vert
  ${PROJECT_SOURCE_DIR}/resources/shaders/osgOcean_dof_combiner.frag
  ${PROJECT_SOURCE_DIR}/resources/shaders/osgOcean_water_distortion.vert
  ${PROJECT_SOURCE_DIR}/resources/shaders/osgOcean_water_distortion.frag

  ${PROJECT_SOURCE_DIR}/resources/shaders/osgOcean_ocean_scene.vert
  ${PROJECT_SOURCE_DIR}/resources/shaders/osgOcean_ocean_scene.frag
  ${PROJECT_SOURCE_DIR}/resources/shaders/osgOcean_ocean_scene_lispsm.vert
  ${PROJECT_SOURCE_DIR}/resources/shaders/osgOcean_ocean_scene_lispsm.frag
  
  ${PROJECT_SOURCE_DIR}/resources/shaders/osgOcean_heightmap.vert
  ${PROJECT_SOURCE_DIR}/resources/shaders/osgOcean_heightmap.frag
)


SET( TEXTURE_FILES
  ${PROJECT_SOURCE_DIR}/resources/textures/sea_foam.png
  ${PROJECT_SOURCE_DIR}/resources/textures/sun_glare.png
)

SET( LIB_HEADERS
  ${HEADER_PATH}/Cylinder
  ${HEADER_PATH}/DistortionSurface
  ${HEADER_PATH}/FFTOceanTechnique
  ${HEADER_PATH}/FFTOceanSurface
  ${HEADER_PATH}/FFTOceanSurfaceVBO
  ${HEADER_PATH}/FFTSimulation
  ${HEADER_PATH}/GodRays
  ${HEADER_PATH}/GodRayBlendSurface
  ${HEADER_PATH}/MipmapGeometry
  ${HEADER_PATH}/MipmapGeometryVBO
  ${HEADER_PATH}/OceanScene
  ${HEADER_PATH}/OceanTechnique
  ${HEADER_PATH}/OceanTile
  ${HEADER_PATH}/RandUtils
  ${HEADER_PATH}/ScreenAlignedQuad
  ${HEADER_PATH}/ShaderManager
  ${HEADER_PATH}/SiltEffect
  ${HEADER_PATH}/WaterTrochoids
  ${HEADER_PATH}/Export
  ${HEADER_PATH}/Version
)

# Adds headers to the "Header Files" group in visual studio
SOURCE_GROUP( "Header Files" ${HEADER_PATH}/*.* )

# Adds shaders to the "Shaders" group in visual studio
SOURCE_GROUP( "Shaders" FILES ${SHADER_FILES} )

ADD_DEFINITIONS(-DOSGOCEAN_LIBRARY)

INCLUDE_DIRECTORIES (
   ${OSG_INCLUDE_DIR}
   ${FFT_INCLUDE_DIR}
   ${PROJECT_SOURCE_DIR}/include/
)

ADD_LIBRARY(
  osgOcean
  SHARED
  ${LIB_HEADERS}
  Cylinder.cpp
  DistortionSurface.cpp
  FFTOceanTechnique.cpp
  FFTOceanSurface.cpp
  FFTOceanSurfaceVBO.cpp
  FFTSimulation.cpp
  GodRays.cpp
  GodRayBlendSurface.cpp
  MipmapGeometry.cpp
  MipmapGeometryVBO.cpp
  OceanScene.cpp
  OceanTechnique.cpp
  OceanTile.cpp
  ScreenAlignedQuad.cpp
  ShaderManager.cpp
  SiltEffect.cpp
  WaterTrochoids.cpp
  Version.cpp
  ${SHADER_FILES}
)

SET_TARGET_PROPERTIES(
  osgOcean
  PROPERTIES
  PUBLIC_HEADER "${LIB_HEADERS}"
)

TARGET_LINK_LIBRARIES(
  osgOcean

  optimized ${OSG_LIBRARY}
  debug ${OSG_LIBRARY_DEBUG}
  optimized ${OSGUTIL_LIBRARY}
  debug ${OSGUTIL_LIBRARY_DEBUG}
  optimized ${OSGDB_LIBRARY}
  debug ${OSGDB_LIBRARY_DEBUG}
  optimized ${OSGGA_LIBRARY}
  debug ${OSGGA_LIBRARY_DEBUG}
  optimized ${OPENTHREADS_LIBRARY}
  debug ${OPENTHREADS_LIBRARY_DEBUG}

  ${OPENGL_LIBRARY}
  ${FFT_LIBRARY}
)

INSTALL(
  TARGETS osgOcean
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_PREFIX}/include/uwsim_osgocean/osgOcean
)
