define_module(LIBRARY pysemantic=${MODULE_semantic_VERSION}
  DEPENDS
  semantic=${MODULE_semantic_VERSION}
  pysupport>=1.4
  Boost_THREAD Boost_SYSTEM)

set(pysemantic_SOURCES
  semanticmodule.cpp
  )

add_library(pysemantic STATIC ${pysemantic_SOURCES})
buildsys_library(pysemantic)
target_link_libraries(pysemantic ${MODULE_LIBRARIES})
