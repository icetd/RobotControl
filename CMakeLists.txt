cmake_minimum_required(VERSION 3.14)

project(RobotControl)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Realse)
set(CMKAE_C_FLAGS "${CMKAE_C_FLAGS} -O2")
set(CMKAE_CXX_FLAGS "${CMKAE_CXX_FLAGS} -O2")

find_package(catkin REQUIRED COMPONENTS 
    roscpp
    geometry_msgs
    rviz
)
catkin_package()


#======================= INCLUSION OF Qt =======================#
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_PREFIX_PATH $ENV{QTDIR})
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

find_package(Qt5Core REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5Multimedia REQUIRED)

#=================== INCLUSION OF Project Files ====================#
set(FORMS_DIR   "${CMAKE_SOURCE_DIR}/forms")
set(RES_DIR     "${CMAKE_SOURCE_DIR}/res")
set(INCLUDE_DIR "${CMAKE_SOURCE_DIR}/inc")
set(SOURCE_DIR  "${CMAKE_SOURCE_DIR}/src")


include_directories(
    ${FORMS_DIR}
    ${INCLUDE_DIR}
    ${SOURCE_DIR}
    ${catkin_INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}
 )

file(GLOB_RECURSE SRC_FILES
    "${SOURCE_DIR}/*.cpp"
)

file (GLOB_RECURSE INC_FILES
    "${INCLUDE_DIR}/*.h"
)

file (GLOB_RECURSE FORMS_FILES
    "${FORMS_DIR}/*.ui"
)

file (GLOB_RECURSE RES_FILES
    "${RES_DIR}/Qss/*.qss"
    "${RES_DIR}/Icon/*.*"
    "${RES_DIR}/*.qrc"
)
#=================== SETUP EXECTUABLE ====================#

# Enable debug logging on RELWITHDEBINFO configuration

set_property(DIRECTORY APPEND PROPERTY COMPILE_DEFINITIONS
    $<$<CONFIG:RELWITHDEBINFO>:QT_MESSAGELOGCONTEXT>
)

# Add the forms directory to the AUTOUIC search paths
set(CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_AUTOUIC_SEARCH_PATHS} ${FORMS_DIR})

# Add the forms directory to the AUTORCC serach paths
set(CMAKE_AUTORCC_SEARCH_PATHS ${CMAKE_AUROECC_SEARCH_PATHS} ${RES_DIR})

# Add the executable

add_executable(${PROJECT_NAME} ${SRC_FILES} ${INC_FILES} ${FORMS_FILES} ${RES_FILES})

# Add the target includes for MY_PROJECT 

target_include_directories(${PROJECT_NAME} PRIVATE ${FORMS_DIR})
target_include_directories(${PROJECT_NAME} PRIVATE ${RES_DIR})
target_include_directories(${PROJECT_NAME} PRIVATE ${INCLUDE_DIR})
target_include_directories(${PROJECT_NAME} PRIVATE ${SOURCE_DIR})

#===================== LINKING LIBRARIES =======================#
target_link_libraries(${PROJECT_NAME} 
    Qt5::Core
    Qt5::Widgets
    Qt5::Gui
    Qt5::Multimedia
    ${catkin_LIBRARIES}
 )

