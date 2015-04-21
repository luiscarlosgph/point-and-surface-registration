#/*============================================================================
#
#  research-computing-with-cpp-demo: CMake based demo code. 
#
#  Copyright (c) University College London (UCL). All rights reserved.
#
#  This software is distributed WITHOUT ANY WARRANTY; without even
#  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
#  PURPOSE.
#
#  See LICENSE.txt in the top level directory for details.
#
#============================================================================*/

set(RCCPP_USE_Boost 1)
set(RCCPP_USE_Boost_LIBRARIES "filesystem;system;date_time;regex;thread;iostreams;program_options")

if(RCCPP_USE_Boost)

  # Sanity checks
  if(DEFINED BOOST_ROOT AND NOT EXISTS ${BOOST_ROOT})
    message(FATAL_ERROR "BOOST_ROOT variable is defined but corresponds to non-existing directory")
  endif()

  set(version "1_56_0")
  set(location "${RCCPP_EP_TARBALL_LOCATION}/boost_${version}.tar.gz")

  rccppMacroDefineExternalProjectVariables(Boost ${version} ${location})
  set(proj_DEPENDENCIES)

  string(REPLACE "^^" ";" RCCPP_USE_Boost_LIBRARIES "${RCCPP_USE_Boost_LIBRARIES}")

  if(NOT DEFINED BOOST_ROOT AND NOT RCCPP_USE_SYSTEM_Boost)

    set(_boost_libs )
    set(INSTALL_COMMAND "")

    if(RCCPP_USE_Boost_LIBRARIES)

      # Set the boost root to the libraries install directory
      set(BOOST_ROOT "${proj_INSTALL}")

      # We need binary boost libraries
      string(REPLACE "^^" ";" RCCPP_USE_Boost_LIBRARIES "${RCCPP_USE_Boost_LIBRARIES}")
      foreach(_boost_lib ${RCCPP_USE_Boost_LIBRARIES})
        set(_boost_libs ${_boost_libs} --with-${_boost_lib})
      endforeach()

      if(WIN32)
        set(_boost_variant "")
        set(_shell_extension .bat)
        if(CMAKE_SIZEOF_VOID_P EQUAL 8)
          set(_boost_address_model "address-model=64")
        else()
          set(_boost_address_model "address-model=32")
        endif()
        if(MSVC)
          if(MSVC_VERSION EQUAL 1400)
            set(_boost_toolset "toolset=msvc-8.0")
          elseif(MSVC_VERSION EQUAL 1500)
            set(_boost_toolset "toolset=msvc-9.0")
          elseif(MSVC_VERSION EQUAL 1600)
            set(_boost_toolset "toolset=msvc-10.0")
          elseif(MSVC_VERSION EQUAL 1700)
            set(_boost_toolset "toolset=msvc-11.0")
          endif()
        endif()
      else()
        if(CMAKE_BUILD_TYPE STREQUAL "Debug")
          set(_boost_variant "variant=debug")
        else()
          set(_boost_variant "variant=release")
        endif()
        set(_shell_extension .sh)
      endif()

      if(BUILD_SHARED_LIBS)
        set(_boost_linkage "link=shared")
      else()
        set(_boost_linkage "link=static")
      endif()

      if(APPLE)
        set(APPLE_CMAKE_SCRIPT ${proj_CONFIG}/ChangeBoostLibsInstallNameForMac.cmake)
        configure_file(${CMAKE_CURRENT_SOURCE_DIR}/CMake/CMakeExternals/ChangeBoostLibsInstallNameForMac.cmake.in ${APPLE_CMAKE_SCRIPT} @ONLY)
        set(INSTALL_COMMAND ${CMAKE_COMMAND} -P ${APPLE_CMAKE_SCRIPT})

        # Set OSX_SYSROOT
        set (APPLE_SYSROOT_FLAG)
        if (NOT ${CMAKE_OSX_SYSROOT} STREQUAL "")
          set (APPLE_SYSROOT_FLAG --sysroot=${CMAKE_OSX_SYSROOT})
        endif()

        # Set the boost build command for apple
        set(_boost_build_cmd ${proj_SOURCE}/bjam ${APPLE_SYSROOT_FLAG} --builddir=${proj_BUILD} --prefix=${proj_INSTALL}
            ${_boost_toolset} ${_boost_address_model} ${_boost_variant} ${_boost_libs} ${_boost_linkage} threading=multi runtime-link=shared -q install)
      else()
        set(_boost_build_cmd ${proj_SOURCE}/bjam --build-dir=${proj_BUILD} --prefix=${proj_INSTALL} ${_boost_toolset} ${_boost_address_model}
            ${_boost_variant} ${_boost_libs} ${_boost_linkage}  threading=multi runtime-link=shared -q install)
      endif()

      set(_boost_cfg_cmd ${proj_SOURCE}/bootstrap${_shell_extension})

    else()
      # If no libraries are specified set the boost root to the boost src directory
      set(BOOST_ROOT "${proj_SOURCE}")
      set(_boost_cfg_cmd )
      set(_boost_build_cmd )
    endif()

    ExternalProject_Add(${proj}
      PREFIX ${proj_CONFIG}
      SOURCE_DIR ${proj_SOURCE}
      # Boost needs in-source builds
      BINARY_DIR ${proj_SOURCE}
      INSTALL_DIR ${proj_INSTALL}
      URL ${proj_LOCATION}
      URL_MD5 ${proj_CHECKSUM}
      CONFIGURE_COMMAND "${_boost_cfg_cmd}"
      BUILD_COMMAND "${_boost_build_cmd}"
      INSTALL_COMMAND "${INSTALL_COMMAND}"
      CMAKE_ARGS
        ${EP_COMMON_ARGS}
        ${BOOST_ARGS}
        -DWITH_BZIP2:BOOL=OFF
        -DWITH_DOXYGEN:BOOL=OFF
        -DWITH_EXPAT:BOOL=OFF
        -DWITH_ICU:BOOL=OFF
        -DWITH_PYTHON:BOOL=OFF
        -DWITH_XSLTPROC:BOOL=OFF
        -DWITH_VALGRIND:BOOL=OFF
        -DWITH_ZLIB:BOOL=OFF
        -DCMAKE_INSTALL_PREFIX:PATH=${proj_INSTALL}
      DEPENDS ${proj_DEPENDENCIES}
    )

    set(BOOST_ROOT ${proj_INSTALL})
    set(BOOST_INCLUDEDIR "${BOOST_ROOT}/include")
    set(BOOST_LIBRARYDIR "${BOOST_ROOT}/lib")

    message("SuperBuild loading Boost from ${BOOST_ROOT}")
    message("SuperBuild loading Boost using BOOST_INCLUDEDIR=${BOOST_INCLUDEDIR}")
    message("SuperBuild loading Boost using BOOST_LIBRARYDIR=${BOOST_LIBRARYDIR}")

  else()

    mitkMacroEmptyExternalProject(${proj} "${proj_DEPENDENCIES}")

  endif()

endif()
