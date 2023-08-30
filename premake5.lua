-- premake5.lua
workspace "CameraCalibration"
architecture "x64"
configurations { "Debug", "Release" }
startproject "CamCalibration"

   
outputdir = "%{cfg.buildcfg}-%{cfg.system}-%{cfg.architecture}"

IncludeDir = {}
IncludeDir["OpenCV"] = "vendor/opencv/include"

project "CamCalibration"
   kind "ConsoleApp"
   language "C++"
   cppdialect "C++17"
   targetdir ("bin/" .. outputdir .. "/%{prj.name}")
   objdir ("bin-int/" .. outputdir .. "/%{prj.name}")

   files { 
        "src/**.cpp",
        "src/**.h",
    }

   includedirs {
       "%{IncludeDir.OpenCV}",
   }

   links {
       "./vendor/opencv/lib/opencv_world480d.lib"
   }

   filter { "system:windows" }
--      prelinkcommands { "copy ./vendor/opencv/lib/opencv_world480d.dll %{targetdir}" }

   filter { "not system:windows" }
      prelinkcommands { "cp ./vendor/opencv/lib/opencv_world480d.dll %{targetdir}" }

   filter "configurations:Debug"
      defines { "DEBUG" }
      symbols "On"

   filter "configurations:Release"
      defines { "NDEBUG" }
      optimize "On"