define_module(LIBRARY pysemantic=${MODULE_geo_VERSION}
  DEPENDS
  semantic=${MODULE_semantic_VERSION}
  Boost_THREAD Boost_SYSTEM Boost_PYTHON PYTHONLIBS)

set(pysemantic_SOURCES
  semanticmodule.cpp
  )

add_library(pysemantic STATIC ${pysemantic_SOURCES})
buildsys_library(pysemantic)
target_link_libraries(pysemantic ${MODULE_LIBRARIES})
