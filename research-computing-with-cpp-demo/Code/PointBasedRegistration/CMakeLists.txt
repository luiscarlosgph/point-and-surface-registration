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

add_executable(PointBasedRegistration main.cpp commandlineoptions.cpp)
target_link_libraries(PointBasedRegistration ${ITK_LIBRARIES} ${Boost_LIBRARIES})

add_executable(PointBasedRegistrationUnitTesting testing.cpp commandlineoptions.cpp)
target_link_libraries(PointBasedRegistrationUnitTesting ${ITK_LIBRARIES} ${Boost_LIBRARIES})
