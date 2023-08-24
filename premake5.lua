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
   targetdir "bin/%{cfg.buildcfg}"

   defines {OpenCV_STATIC}

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

   filter "configurations:Debug"
      defines { "DEBUG" }
      symbols "On"

   filter "configurations:Release"
      defines { "NDEBUG" }
      optimize "On"