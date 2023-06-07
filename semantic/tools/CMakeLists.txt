define_module(BINARY semantic2obj
  DEPENDS semantic service utility
  )

# Semantic world to OBJ encoder.
set(semantic2obj_SOURCES
  semantic2obj.cpp
  )

add_executable(semantic2obj ${semantic2obj_SOURCES})
target_link_libraries(semantic2obj ${MODULE_LIBRARIES})
target_compile_definitions(semantic2obj PRIVATE ${MODULE_DEFINITIONS})
buildsys_binary(semantic2obj)

define_module(BINARY semantic2semantic
  DEPENDS semantic service utility
  )
# Simple format checker, load and saves semantic world file.
set(semantic2semantic_SOURCES
  semantic2semantic.cpp
  )

add_executable(semantic2semantic ${semantic2semantic_SOURCES})
target_link_libraries(semantic2semantic ${MODULE_LIBRARIES})
target_compile_definitions(semantic2semantic PRIVATE ${MODULE_DEFINITIONS})
buildsys_binary(semantic2semantic)

define_module(BINARY semantic2gpkg
  DEPENDS semantic service utility
  )
# semantic world to (semantic) GeoPackage
set(semantic2gpkg_SOURCES
  semantic2gpkg.cpp
  )

add_executable(semantic2gpkg ${semantic2gpkg_SOURCES})
target_link_libraries(semantic2gpkg ${MODULE_LIBRARIES})
target_compile_definitions(semantic2gpkg PRIVATE ${MODULE_DEFINITIONS})
buildsys_binary(semantic2gpkg)

define_module(BINARY gpkg2semantic
  DEPENDS semantic service utility
  )
# (semantic) GeoPackage to semantic world
set(gpkg2semantic_SOURCES
  gpkg2semantic.cpp
  )

add_executable(gpkg2semantic ${gpkg2semantic_SOURCES})
target_link_libraries(gpkg2semantic ${MODULE_LIBRARIES})
target_compile_definitions(gpkg2semantic PRIVATE ${MODULE_DEFINITIONS})
buildsys_binary(gpkg2semantic)


# On-request installation
if(semantic-tools-INSTALL-COMPONENT)
  install(TARGETS
    semantic2obj semantic2semantic semantic2gpkg gpkg2semantic
    RUNTIME DESTINATION bin
    COMPONENT ${semantic-tools-INSTALL-COMPONENT})
endif()

if (TARGET vef)
  define_module(BINARY semantic2vef
    DEPENDS vef semantic service utility
    )

  # Semantic world to vef converter
  set(semantic2vef_SOURCES
    semantic2vef.cpp
    )

  add_executable(semantic2vef ${semantic2vef_SOURCES})
  target_link_libraries(semantic2vef ${MODULE_LIBRARIES} vef)
  target_compile_definitions(semantic2vef PRIVATE ${MODULE_DEFINITIONS})
  buildsys_binary(semantic2vef)

  # On-request installation
  if(semantic-tools-INSTALL-COMPONENT)
    install(TARGETS
      semantic2vef
      RUNTIME DESTINATION bin
      COMPONENT ${semantic-tools-INSTALL-COMPONENT})
  endif()
endif()
