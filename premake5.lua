include "Dependencies.lua"

workspace "PanoramicCameraPose"
    architecture "x64"

    configurations
    {
        "Release"
    }
	startproject "PanoramicCameraPose"

outputdir = "%{cfg.buildcfg}-%{cfg.system}-%{cfg.architecture}"

-- ImGui premake static lib
include "PanoramicCameraPose/vendor/ImGui/include/imgui"

project "PanoramicCameraPose"
    location "PanoramicCameraPose"
    kind "ConsoleApp"
    language "C++"
    cppdialect "C++17"
	staticruntime "off" -- for multithread-specific and DLL-specific version of the run-time library
    targetdir ("bin/" .. outputdir .. "/%{prj.name}")
    objdir ("bin-int/" .. outputdir .. "/%{prj.name}")
    ignoredefaultlibraries { "LIBCMT" }

     -- precompiled header
    pchheader "pch.h"
    pchsource "%{prj.name}/src/pch.cpp"
   
    files
    {
        "%{prj.name}/src/**.h",
        "%{prj.name}/src/**.cpp",
		"%{prj.name}/src/glad.c",
        "%{prj.name}/vendor/stb_image/**.h",
		"%{prj.name}/vendor/stb_image/**.cpp",
    }

    includedirs
    {
        "%{prj.name}/src",
        "%{IncludeDir.Glad}",
        "%{IncludeDir.glfw}",
        "%{IncludeDir.glm}",
        "%{IncludeDir.ImGui}",
        "%{IncludeDir.eigen}",
        "%{IncludeDir.stb_image}",
        "%{IncludeDir.OpenMVG}",
		"%{IncludeDir.OpenCV}",
		"C:/gurobi952/win64/include"
    }
	
    --filter {"files:%{prj.name}/vecdor/imgui/include/imgui/**.cpp or %{prj.name}/vendor/stb_image/**.cpp or %{prj.name}/src/glad.c"}
	    --flags {"NoPCH"}
    filter "files:PanoramicCameraPose/src/glad.c"
        flags { "NoPCH" }

    filter "system:windows"
		systemversion "latest"

    filter "configurations:Release"
        
        defines 
        {
            "_CRT_SECURE_NO_WARNINGS",
            "_WINDOWS",
            "NDEBUG",
            "NOMINMAX",
            "_USE_MATH_DEFINES",
            "OPENMVG_USE_AVX",
            "_CONSOLE",
            "GLOG_NO_ABBREVIATED_SEVERITIES",
            "CERES_USE_CXX_THREADS",
            "CERES_MSVC_USE_UNDERSCORE_PREFIXED_BESSEL_FUNCTIONS",
            "__SSE2__",
            "__SSE3__",
            "__SSSE3__",
            "__SSE4_1__",
            "__SSE4_2__",
            "__AVX__",
            "OPENMVG_USE_OPENMP",
            "USE_PATENTED_LIGT",
            "LEMON_ONLY_TEMPLATES"
        }

        runtime "Release"
		optimize "on"
		
		libdirs
		{
			"%{prj.name}/vendor/OpenMVG/lib/Release",
			"%{prj.name}/vendor/glfw/lib/Release",
			"%{prj.name}/vendor/opencv/lib",
			"C:/gurobi952/win64/lib"
		}
		
        links
        {
            "opengl32.lib",
            "glfw3.lib",
			"ImGui",
            "openMVG_fast.lib",
            "openMVG_features.lib",
            "openMVG_image.lib",
            "openMVG_matching.lib",
            "openMVG_multiview.lib",
            "openMVG_numeric.lib",
            "openMVG_robust_estimation.lib",
			"openMVG_sfm.lib",
			"openMVG_system.lib",
			"openMVG_stlplus.lib",
			"openMVG_geometry.lib",
            "ceres.lib",
            "gflags.lib",
            "glog.lib",
            "jpeg.lib", 
            "libpng16.lib",
            "tiff.lib",
            "zlib.lib",
			"gurobi95.lib",
			"gurobi_c++md2019.lib",
			"opencv_world455.lib"
        }
		
		postbuildcommands 
		{ 
			"{COPYFILE} %{wks.location}/%{prj.name}/vendor/OpenMVG/bin/Release/*.dll %{wks.location}bin/" .. outputdir .. "/%{prj.name}/",
			"{COPYFILE} %{wks.location}/%{prj.name}/vendor/opencv/bin/opencv_world455.dll %{wks.location}bin/" .. outputdir .. "/%{prj.name}/"
		}
            
    