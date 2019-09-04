ue camera

## Add Remotes

    $ conan remote add camposs "https://conan.campar.in.tum.de/api/conan/conan-camposs"
    $ conan remote add ubitrack "https://conan.campar.in.tum.de/api/conan/conan-ubitrack"

## For Users: Use this package

### Basic setup

    $ mkdir build
	$ cd build
	$ conan install ..
	$ conan build .. -pf ../example

	$ cd example
	$ conan install conan/conanfile.py

	$ activate_run.bat/sh
	$ utGLFWConsole130 test_uecamera.dfg

