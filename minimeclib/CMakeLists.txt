# Lines that begin with a # are comments
# set the minimum required version of cmake, usually the first line
cmake_minimum_required(VERSION 3.22)

# project_name sets the name of the project and causes cmake to
# find the c and c++ compilers
project(minimeclib)

# Find dependencies.
# Many libraries ship with files that allow CMake to find them
# Then general behavior is to call "find_package" but the options
# provided are package specific.  Usually there is then a CMAKE variable
# That is defined to reference the library
find_package(Doxygen)
find_package(tf2 REQUIRED)

include(CTest)

# CTest sets BUILD_TESTING to on. To disable tests add -DBUILD_TESTING=OFF when invoking cmake
if(BUILD_TESTING)
    # Find the Unit testing framework. In this example, Catch2
    find_package(Catch2 3 REQUIRED)

    # A test is just an executable that is linked against the unit testing library
    # add_executable(minimeclib_test tests/test_geometry2d.cpp tests/test_se2d.cpp tests/test_svg.cpp tests/test_diff_drive.cpp)
    # target_link_libraries(minimeclib_test Catch2::Catch2WithMain minimeclib)

    # register the test with CTest, telling it what executable to run
    # add_test(NAME minimeclib_test COMMAND minimeclib_test)

endif()

# Building documentation should be optional.
# To build documentation pass -DBUILD_DOCS=ON when generating the build system
option(BUILD_DOCS "Build the documentation" OFF)

# build just because Doxygen is missing
if(${DOXYGEN_FOUND} AND ${BUILD_DOCS})
    # Turn the README.md into the homepage of the doxygen docs
    set(DOXYGEN_USE_MDFILE_AS_MAINPAGE README.md)

    # Tell Doxygen where to find the documentation
    doxygen_add_docs(doxygen include/ src/ README.md ALL)

    # The documentation will be in the build/html directory
    # The main page is build/html/index.html
endif()

# Create a library.  Can specify if it is shared or static but usually
# you don't need or want to.
# name is the name of the library without the extension or lib prefix
# name creates a cmake "target"
add_library(minimeclib src/kinematics.cpp)
target_link_libraries(minimeclib tf2::tf2)
# Use target_include_directories so that #include"mylibrary/header.hpp" works
# The use of the <BUILD_INTERFACE> and <INSTALL_INTERFACE> is because when
# Using the library from the build directory or after installation
# During build, the headers are read from the source code directory
# When used from the installed location, headers are in the
# system include/ directory
target_include_directories(minimeclib
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

# specify additional compilation flags for the library
# Public causes the flags to propagate to anything
# that links against this library
target_compile_options(minimeclib PUBLIC -Wall -Wextra -pedantic)

# Enable c++17 support.
# Public causes the features to propagate to anything
# that links against this library
target_compile_features(minimeclib PUBLIC cxx_std_17)

# Create an executable from the following source code files
# The Name of the executable creates a cmake "target"
# add_executable()

# Use target_link_libraries to add dependencies to a "target"
# (e.g., a library or executable)
# This will automatically add all required library files
# that need to be linked
# and paths to th locations of header files
# target_link_libraries(frame_main minimeclib)

# install the include files by copying the whole include directory
install(DIRECTORY include/minimeclib DESTINATION include)

# Create a CMake Exported Target containing the lib and exe.
# Also create CMake Export called projet_name-targets
# The CMake Export contains files that allow other CMake projects
# to find this project. It must be installed separately.
install(TARGETS minimeclib EXPORT minimeclib-targets)

# The minimeclib-targets created by install(TARGETS) needs to be installed.
# install(EXPORT ...) will generate a file called minimeclib-config.cmake
# that contains the exported targets.
# After installation this file will then be found when calling
# find_package(minimeclib) from another cmake project
# A user can then target_link_libraries(target minimeclib::library)
# to use the libraries installed here
install(EXPORT minimeclib-targets
        FILE minimeclib-config.cmake
        NAMESPACE minimeclib::
        DESTINATION lib/cmake/${PROJECT_NAME})