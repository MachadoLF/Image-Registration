# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.17

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Disable VCS-based implicit rules.
% : %,v


# Disable VCS-based implicit rules.
% : RCS/%


# Disable VCS-based implicit rules.
% : RCS/%,v


# Disable VCS-based implicit rules.
% : SCCS/s.%


# Disable VCS-based implicit rules.
% : s.%


.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = "/home/leo/Desktop/Doutorado/Registration Modules/MultiStage/MultiStage_raw"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "/home/leo/Desktop/Doutorado/Registration Modules/MultiStage/MultiStage_raw/debug"

# Include any dependencies generated for this target.
include CMakeFiles/MultiStageImageRegistration1.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/MultiStageImageRegistration1.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/MultiStageImageRegistration1.dir/flags.make

CMakeFiles/MultiStageImageRegistration1.dir/MultiStageImageRegistration1.cxx.o: CMakeFiles/MultiStageImageRegistration1.dir/flags.make
CMakeFiles/MultiStageImageRegistration1.dir/MultiStageImageRegistration1.cxx.o: ../MultiStageImageRegistration1.cxx
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/leo/Desktop/Doutorado/Registration Modules/MultiStage/MultiStage_raw/debug/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/MultiStageImageRegistration1.dir/MultiStageImageRegistration1.cxx.o"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/MultiStageImageRegistration1.dir/MultiStageImageRegistration1.cxx.o -c "/home/leo/Desktop/Doutorado/Registration Modules/MultiStage/MultiStage_raw/MultiStageImageRegistration1.cxx"

CMakeFiles/MultiStageImageRegistration1.dir/MultiStageImageRegistration1.cxx.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/MultiStageImageRegistration1.dir/MultiStageImageRegistration1.cxx.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/leo/Desktop/Doutorado/Registration Modules/MultiStage/MultiStage_raw/MultiStageImageRegistration1.cxx" > CMakeFiles/MultiStageImageRegistration1.dir/MultiStageImageRegistration1.cxx.i

CMakeFiles/MultiStageImageRegistration1.dir/MultiStageImageRegistration1.cxx.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/MultiStageImageRegistration1.dir/MultiStageImageRegistration1.cxx.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/leo/Desktop/Doutorado/Registration Modules/MultiStage/MultiStage_raw/MultiStageImageRegistration1.cxx" -o CMakeFiles/MultiStageImageRegistration1.dir/MultiStageImageRegistration1.cxx.s

# Object files for target MultiStageImageRegistration1
MultiStageImageRegistration1_OBJECTS = \
"CMakeFiles/MultiStageImageRegistration1.dir/MultiStageImageRegistration1.cxx.o"

# External object files for target MultiStageImageRegistration1
MultiStageImageRegistration1_EXTERNAL_OBJECTS =

MultiStageImageRegistration1: CMakeFiles/MultiStageImageRegistration1.dir/MultiStageImageRegistration1.cxx.o
MultiStageImageRegistration1: CMakeFiles/MultiStageImageRegistration1.dir/build.make
MultiStageImageRegistration1: /usr/local/lib/libitkdouble-conversion-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libitksys-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libitkvnl_algo-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libitkvnl-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libitkv3p_netlib-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libitknetlib-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libitkvcl-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKCommon-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libitkNetlibSlatec-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKStatistics-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKTransform-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKMesh-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libitkzlib-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKMetaIO-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKSpatialObjects-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKPath-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKLabelMap-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKQuadEdgeMesh-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKIOImageBase-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKOptimizers-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKPolynomials-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKBiasCorrection-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKDICOMParser-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKEXPAT-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libitkgdcmDICT-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libitkgdcmMSFF-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKznz-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKniftiio-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKgiftiio-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libitkhdf5_cpp.a
MultiStageImageRegistration1: /usr/local/lib/libitkhdf5.a
MultiStageImageRegistration1: /usr/local/lib/libITKIOBMP-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKIOBioRad-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKIOBruker-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKIOCSV-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKIOGDCM-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKIOIPL-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKIOGE-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKIOGIPL-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKIOHDF5-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libitkjpeg-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKIOJPEG-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libitkopenjpeg-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKIOJPEG2000-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libitktiff-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKIOTIFF-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKIOLSM-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libitkminc2-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKIOMINC-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKIOMRC-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKIOMeshBase-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKIOMeshBYU-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKIOMeshFreeSurfer-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKIOMeshGifti-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKIOMeshOBJ-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKIOMeshOFF-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKIOMeshVTK-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKIOMeta-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKIONIFTI-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKNrrdIO-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKIONRRD-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libitkpng-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKIOPNG-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKIOSiemens-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKIOXML-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKIOSpatialObjects-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKIOStimulate-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKTransformFactory-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKIOTransformBase-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKIOTransformHDF5-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKIOTransformInsightLegacy-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKIOTransformMatlab-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKIOVTK-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKKLMRegionGrowing-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libitklbfgs-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKOptimizersv4-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKTestKernel-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKVTK-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKVideoCore-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKVideoIO-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKWatersheds-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libitkopenjpeg-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libitkminc2-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKIOIPL-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKIOXML-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libitkhdf5_cpp.a
MultiStageImageRegistration1: /usr/local/lib/libitkhdf5.a
MultiStageImageRegistration1: /usr/local/lib/libITKIOTransformBase-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKTransformFactory-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKOptimizers-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libitklbfgs-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKIOBMP-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKIOGDCM-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libitkgdcmMSFF-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libitkgdcmDICT-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libitkgdcmIOD-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libitkgdcmDSED-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libitkgdcmCommon-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libitkgdcmjpeg8-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libitkgdcmjpeg12-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libitkgdcmjpeg16-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libitkgdcmopenjp2-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libitkgdcmcharls-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libitkgdcmuuid-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKIOGIPL-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKIOJPEG-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKIOTIFF-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libitktiff-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libitkjpeg-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKIOMeshBYU-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKIOMeshFreeSurfer-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKIOMeshGifti-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKgiftiio-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKEXPAT-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKIOMeshOBJ-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKIOMeshOFF-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKIOMeshVTK-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKIOMeshBase-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKQuadEdgeMesh-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKIOMeta-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKMetaIO-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKIONIFTI-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKniftiio-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKznz-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKIONRRD-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKNrrdIO-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKIOPNG-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libitkpng-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libitkzlib-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKIOVTK-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKIOImageBase-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKVideoCore-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKStatistics-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libitkNetlibSlatec-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKSpatialObjects-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKMesh-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKTransform-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKPath-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKCommon-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libitkdouble-conversion-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libitksys-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libITKVNLInstantiation-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libitkvnl_algo-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libitkvnl-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libitkv3p_netlib-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libitknetlib-5.0.a
MultiStageImageRegistration1: /usr/local/lib/libitkvcl-5.0.a
MultiStageImageRegistration1: CMakeFiles/MultiStageImageRegistration1.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="/home/leo/Desktop/Doutorado/Registration Modules/MultiStage/MultiStage_raw/debug/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable MultiStageImageRegistration1"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/MultiStageImageRegistration1.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/MultiStageImageRegistration1.dir/build: MultiStageImageRegistration1

.PHONY : CMakeFiles/MultiStageImageRegistration1.dir/build

CMakeFiles/MultiStageImageRegistration1.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/MultiStageImageRegistration1.dir/cmake_clean.cmake
.PHONY : CMakeFiles/MultiStageImageRegistration1.dir/clean

CMakeFiles/MultiStageImageRegistration1.dir/depend:
	cd "/home/leo/Desktop/Doutorado/Registration Modules/MultiStage/MultiStage_raw/debug" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/home/leo/Desktop/Doutorado/Registration Modules/MultiStage/MultiStage_raw" "/home/leo/Desktop/Doutorado/Registration Modules/MultiStage/MultiStage_raw" "/home/leo/Desktop/Doutorado/Registration Modules/MultiStage/MultiStage_raw/debug" "/home/leo/Desktop/Doutorado/Registration Modules/MultiStage/MultiStage_raw/debug" "/home/leo/Desktop/Doutorado/Registration Modules/MultiStage/MultiStage_raw/debug/CMakeFiles/MultiStageImageRegistration1.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : CMakeFiles/MultiStageImageRegistration1.dir/depend

