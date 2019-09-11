# All cpp files for unreal camera playback
set(UBITRACK_LIBRARY_NAME "uecameraplayer")

FILE(GLOB UBITRACK_SRCS RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
   "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
)

FILE(GLOB UBITRACK_HDRS RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
        "${CMAKE_CURRENT_SOURCE_DIR}/*.h"
        )

add_library(${UBITRACK_LIBRARY_NAME} SHARED ${UBITRACK_SRCS} ${UBITRACK_HDRS})


if(MSVC)
	target_link_libraries(${UBITRACK_LIBRARY_NAME}
		CONAN_PKG::ubitrack_core
		CONAN_PKG::ubitrack_vision
		CONAN_PKG::ubitrack_dataflow
	)
else()
	target_link_libraries(${UBITRACK_LIBRARY_NAME}
		CONAN_PKG::ubitrack_core
		CONAN_PKG::ubitrack_vision
		CONAN_PKG::ubitrack_dataflow
		"stdc++fs"
	)
endif()

target_include_directories(${UBITRACK_LIBRARY_NAME} PUBLIC
		../../include
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<BUILD_INTERFACE:${CONAN_INCLUDE_DIRS}>
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# do not create symlinks to versioned names of component libraries
set_target_properties(${UBITRACK_LIBRARY_NAME} PROPERTIES NO_SONAME ON)

# Apply Target Properties
if(MSVC)
  if(CMAKE_CROSSCOMPILING)
    set_target_properties(${UBITRACK_LIBRARY_NAME} PROPERTIES LINK_FLAGS "/NODEFAULTLIB:secchk")
  endif()
  set_target_properties(${UBITRACK_LIBRARY_NAME} PROPERTIES LINK_FLAGS "/NODEFAULTLIB:libc /DEBUG")
endif()

ubitrack_set_target_properties(${UBITRACK_LIBRARY_NAME})
ubitrack_install_component(${UBITRACK_LIBRARY_NAME})
