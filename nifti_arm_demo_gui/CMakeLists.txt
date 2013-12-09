cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Debug)

rosbuild_init()


#Adding QT-Support
find_package(Qt4 REQUIRED)
# enable/disable some Qt features
set( QT_USE_QTGUI TRUE )
include(${QT_USE_FILE})

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#MOC-Support
set(qt_srcs
      src/msgthread.cpp
      src/niftiarmgui.cpp
      src/RosCalls.cpp
      src/main.cpp
   )
set(qt_hdrs
      src/msgthread.h
      src/RosCalls.h
      src/niftiarmgui.h
    )
qt4_automoc(${qt_srcs})
QT4_WRAP_CPP(qt_moc_srcs ${qt_hdrs})
#Processing the UI-Files
QT4_WRAP_UI(uis_h ui/niftiarmgui.ui)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

#If uncommented you'll need to replace slots by Q_SLOTS; If you do not uncomment it you can write normal Qt-code
#ADD_DEFINITIONS(-DQT_NO_KEYWORDS)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_executable(gui src/main.cpp
    ${uis_h} ${qt_srcs} ${qt_moc_srcs})
target_link_libraries(gui ${QT_LIBRARIES})
