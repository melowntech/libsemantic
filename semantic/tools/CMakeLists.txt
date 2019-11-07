define_module(BINARY semantic2obj
  DEPENDS semantic service utility
  )

# Semantic world to OBJ encoder.
set(semantic2obj_SOURCES
  semantic2obj.cpp
  )

file_to_cpp(semantic2obj_SOURCES
  semantic2obj::semantic_mtl
  data/semantic.mtl)

add_executable(semantic2obj ${semantic2obj_SOURCES})
target_link_libraries(semantic2obj ${MODULE_LIBRARIES})
buildsys_target_compile_definitions(semantic2obj ${MODULE_DEFINITIONS})
buildsys_binary(semantic2obj)

# Simple format checker, load and saves semantic world file.
set(semantic2semantic_SOURCES
  semantic2semantic.cpp
  )

add_executable(semantic2semantic ${semantic2semantic_SOURCES})
target_link_libraries(semantic2semantic ${MODULE_LIBRARIES})
buildsys_target_compile_definitions(semantic2semantic ${MODULE_DEFINITIONS})
buildsys_binary(semantic2semantic)

# semantic world to (semantic) GeoPackage
set(semantic2gpkg_SOURCES
  semantic2gpkg.cpp
  )

add_executable(semantic2gpkg ${semantic2gpkg_SOURCES})
target_link_libraries(semantic2gpkg ${MODULE_LIBRARIES})
buildsys_target_compile_definitions(semantic2gpkg ${MODULE_DEFINITIONS})
buildsys_binary(semantic2gpkg)

# (semantic) GeoPackage to semantic world
set(gpkg2semantic_SOURCES
  gpkg2semantic.cpp
  )

add_executable(gpkg2semantic ${gpkg2semantic_SOURCES})
target_link_libraries(gpkg2semantic ${MODULE_LIBRARIES})
buildsys_target_compile_definitions(gpkg2semantic ${MODULE_DEFINITIONS})
buildsys_binary(gpkg2semantic)

# On-request installation
if(semantic-tools-INSTALL-COMPONENT)
  install(TARGETS
    semantic2obj semantic2semantic semantic2gpkg gpkg2semantic
    RUNTIME DESTINATION bin
    COMPONENT ${semantic-tools-INSTALL-COMPONENT})
endif()
