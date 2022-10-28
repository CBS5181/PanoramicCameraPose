workspace "PanoramicCameraPose"
    architecture "x64"

    configurations
    {
        "Release"
    }

outputdir = "%{cfg.buildcfg}-%{cfg.system}-%{cfg.architecture}"

project "PanoramicCameraPose"
    location "PanoramicCameraPose"
    kind "ConsoleApp"
    language "C++"
    cppdialect "C++17"
    targetdir ("bin/" .. outputdir .. "/%{prj.name}")
    objdir ("bin-int/" .. outputdir .. "/%{prj.name}")

    files
    {
        "%{prj.name}/src/**.h",
        "%{prj.name}/src/**.cpp",
		"%{prj.name}/src/glad.c",
		"%{prj.name}/vendor/imgui/include/**.h",
		"%{prj.name}/vendor/imgui/include/**.cpp",
        "%{prj.name}/vendor/stb_image/**.h",
		"%{prj.name}/vendor/stb_image/**.cpp"
    }

    includedirs
    {
        "%{prj.name}/src",
        "%{prj.name}/vendor/Glad/include",
        "%{prj.name}/vendor/glfw/include",
        "%{prj.name}/vendor/glm/include",
        "%{prj.name}/vendor/imgui/include",
        "%{prj.name}/vendor/eigen",
        "%{prj.name}/vendor/stb_image",
        "%{prj.name}/vendor/openMVG/include"
    }

    filter "system:windows"
		systemversion "latest"

    filter "configurations:Release"
        defines 
        {
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
			"%{prj.name}/vendor/glfw/lib/Release"
		}
		
        links
        {
            "opengl32.lib",
            "glfw3.lib",
            "openMVG_fast.lib",
            "openMVG_features.lib",
            "openMVG_image.lib",
            "openMVG_matching.lib",
            "openMVG_multiview.lib",
            "openMVG_numeric.lib",
            "openMVG_robust_estimation.lib",
            "ceres.lib",
            "gflags.lib",
            "glog.lib",
            "jpeg.lib", 
            "libpng16.lib",
            "tiff.lib",
            "zlib.lib"
        }
		
		postbuildcommands 
		{ 
			"{COPYFILE} %{wks.location}/%{prj.name}/vendor/OpenMVG/bin/Release/*.dll %{wks.location}bin/" .. outputdir .. "/%{prj.name}/"
		}
            